
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

#include "deck.h"
#include "log.h"
#include "param.h"

#include "vl53l8cx11_ver3.h"
#include "vl53l8cx_api.h"
#include "platform.h"           /* VL53L8CX_Platform */
#include "static_mem.h"
/* Deck SPI and GPIO APIs */
#include "deck_spi.h"
#include "deck_digital.h"
#include "deck_constants.h"
#include "stm32f4xx_spi.h"  /* For SPI_BaudRatePrescaler_* definitions */

/* ===== Memory/RAM strategy toggles =====
 * VL11_USE_CCM: place large non-DMA buffers in CCM (64KB fast SRAM, non-DMA) to free main SRAM.
 * VL11_DEFAULT_RATE_HZ: per-sensor ranging frequency (before multiplexing).
 */
#ifndef VL11_USE_CCM
#define VL11_USE_CCM 1
#endif
#ifndef VL11_DEFAULT_RATE_HZ
#define VL11_DEFAULT_RATE_HZ 10
#endif
#include "vl11_arena.h"

/* Extern-only declarations of the ULD blobs. Must be defined exactly once elsewhere. */
#include "vl53l8cx_buffers.h"


/* ===== Provide your REAL CS pins here ===== */
const deckPin_t g_vl11_cs[VL11_NUM_SENSORS] = {
  (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0},
  (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0},
};

/* ===== Driver state (ultra-low RAM) ===== */
static volatile uint8_t g_running = 0;
static uint8_t  g_rate_hz = VL11_DEFAULT_RATE_HZ;
static uint8_t  g_testGen = 1;
// static uint8_t  g_inited  = 0;
static uint32_t g_tick    = 0;

/* Heap snap + diagnostics */
static uint8_t  g_heapSnap = 0;
static uint8_t  g_blobsOk  = 0;   /* 1 if blobs look valid & in flash */
// static uint8_t  g_initOk[VL11_NUM_SENSORS] = {0};

/* Build-time switch to bypass ULD init for isolation tests (0 = skip, 1 = call init) */
#ifndef VL11_CALL_INIT
#define VL11_CALL_INIT 1
#endif

/* ONE shared configuration + ONE shared results */
NO_DMA_CCM_SAFE_ZERO_INIT static VL53L8CX_Configuration g_dev;
#ifdef VL11_USE_CCM
__attribute__((section(".ccmram")))
#endif
// static VL53L8CX_ResultsData g_res;

/* public last distances */
static uint16_t g_ranges_mm[VL11_NUM_SENSORS];

/* ===== Heap probe helper ===== */
static inline void heapSnap(const char* tag) {
  size_t cur = xPortGetFreeHeapSize();
  size_t min = xPortGetMinimumEverFreeHeapSize();
  DEBUG_PRINT("HEAP[%s] cur=%u minEver=%u\n", (tag?tag:""), (unsigned)cur, (unsigned)min);
}

/* ===== CS helpers ===== */
static inline void cs_low(int i)  { VL11_GPIO_WRITE(g_vl11_cs[i], LOW); }
static inline void cs_high(int i) { VL11_GPIO_WRITE(g_vl11_cs[i], HIGH); }

/* ===== ULD transport over SPI ===== */
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform* p, uint16_t reg, uint8_t value) {
  const int i = (int)p->address;
  VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
  uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
  VL11_SPI_SEND(2, hdr); VL11_SPI_SEND(1, &value);
  cs_high(i); VL11_SPI_RELEASE(); return 0;
}
uint8_t VL53L8CX_RdByte(VL53L8CX_Platform* p, uint16_t reg, uint8_t* p_value) {
  const int i = (int)p->address;
  VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
  uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
  VL11_SPI_SEND(2, hdr); VL11_SPI_RECV(1, p_value);
  cs_high(i); VL11_SPI_RELEASE(); return 0;
}
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform* p, uint16_t reg, uint8_t* p_values, uint32_t size) {
  const int i = (int)p->address;
  VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
  uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
  VL11_SPI_SEND(2, hdr); if (size) VL11_SPI_SEND(size, p_values);
  cs_high(i); VL11_SPI_RELEASE(); return 0;
}
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform* p, uint16_t reg, uint8_t* p_values, uint32_t size) {
  const int i = (int)p->address;
  // VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
  uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
  VL11_SPI_SEND(2, hdr); if (size) VL11_SPI_RECV(size, p_values);
  cs_high(i); VL11_SPI_RELEASE(); return 0;
}
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform* p, uint32_t TimeMs) { (void)p; vTaskDelay(pdMS_TO_TICKS(TimeMs)); return 0; }
void    VL53L8CX_SwapBuffer(uint8_t* buffer, uint16_t size) { (void)buffer; (void)size; }

// /* ===== pick a representative distance ===== */
// static uint16_t pick_distance_mm(const VL53L8CX_ResultsData* r) {
//   const int zones = 16; /* using 4x4 */
//   uint16_t best = 0xFFFF;
//   for (int z = 0; z < zones; z++) {
//     if (r->nb_target_detected[z] > 0) {
//       uint16_t d = r->distance_mm[z * VL53L8CX_NB_TARGET_PER_ZONE + 0];
//       if (d && d < best) best = d;
//     }
//   }
//   return (best == 0xFFFF) ? 0 : best;
// }

// /* ===== worker ===== */
// static void vl11Task(void* arg) {
//   (void)arg;
//   heapSnap("task-start");

//   /* Setup CS pins idle high */
//   for (int i = 0; i < VL11_NUM_SENSORS; i++) {
//     VL11_GPIO_INIT_OUTPUT(g_vl11_cs[i]);
//     VL11_GPIO_WRITE(g_vl11_cs[i], HIGH);
//   }
//   heapSnap("after-cs-setup");

//   /* init sensors sequentially (shared g_dev) */
//   for (int i = 0; i < VL11_NUM_SENSORS && g_running; i++) {
//     heapSnap("before-sensor-init");
//     memset(&g_dev, 0, sizeof(g_dev));
//     g_dev.platform.address = (uint16_t)i;

// #if VL11_CALL_INIT
//     if (g_blobsOk) {
//       vl11_arena_reset(12*1024);   // start with 12 KB; adjust if you see OOM log
//       int8_t st = vl53l8cx_init(&g_dev);
//       g_initOk[i] = (st == VL53L8CX_STATUS_OK) ? 1 : (uint8_t)st;  /* store error code too */
//       DEBUG_PRINT("vl8cx[%d] init -> %d\n", i, (int)st);
//       if (st == VL53L8CX_STATUS_OK) {
//         /* Force 4x4 to shrink result buffers */
//         vl53l8cx_set_resolution(&g_dev, VL53L8CX_RESOLUTION_4X4);
//         /* If your ULD exposes it, keep 1 target/zone (API name differs across drops)
//            Examples:
//            // vl53l8cx_set_nb_target_per_zone(&g_dev, 1);
//            // vl53l8cx_set_nb_targets_per_zone(&g_dev, 1);
//         */
//         vl53l8cx_set_ranging_frequency_hz(&g_dev, g_rate_hz);
//         (void)vl53l8cx_start_ranging(&g_dev);
//       }
//     } else {
//       DEBUG_PRINT("vl8cx[%d] skipped init (blobs missing)\n", i);
//     }
// #else
//     /* Skip ULD init for isolation */
//     vTaskDelay(pdMS_TO_TICKS(2));
// #endif

//     heapSnap("after-sensor-init");
//     vTaskDelay(pdMS_TO_TICKS(5));
//   }
//   g_inited = 1;
//   heapSnap("after-all-init");

//   for (;;) {
//     if (g_heapSnap) { g_heapSnap = 0; heapSnap("param"); }

//     if (g_running && !g_testGen) {
//       for (int i = 0; i < VL11_NUM_SENSORS; i++) {
//         g_dev.platform.address = (uint16_t)i;
//         uint8_t ready = 0;
//         if (vl53l8cx_check_data_ready(&g_dev, &ready) == VL53L8CX_STATUS_OK && ready) {
//           if (vl53l8cx_get_ranging_data(&g_dev, &g_res) == VL53L8CX_STATUS_OK) {
//             g_ranges_mm[i] = pick_distance_mm(&g_res);
//           }
//         }
//       }
//     } else if (g_testGen) {
//       for (int i = 0; i < VL11_NUM_SENSORS; i++) {
//         g_ranges_mm[i] = (uint16_t)(200 + ((g_tick * 7 + i * 93) % 1500));
//       }
//     }

//     g_tick++;
//     vTaskDelay(pdMS_TO_TICKS(VL11_POLL_PERIOD_MS));
//   }
// }

/* Task handle (created on enable toggle) */
// static TaskHandle_t g_task = NULL;

static void onEnableUpdated() {
  heapSnap(g_running ? "enable=1" : "enable=0");
  memset(&g_dev, 0, sizeof(g_dev));
  g_dev.platform.address = (uint16_t)0;
  // uint8_t status = VL53L8CX_STATUS_OK;
  DEBUG_PRINT("\n\n\nWrByte start!!!!!!!!!!!!\n");
  /* snapshot heap before heavy loop */
  heapSnap("before-loop");
  // 低レベルSPI通信テスト: CS pin と SPI 直接制御
  DEBUG_PRINT("VL11: Low-level SPI test starting...\n");
  
  // CS pins の初期化 (IO3, IO4 を OUTPUT mode にして HIGH に設定)
  pinMode(DECK_GPIO_IO3, OUTPUT);
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO3, HIGH);
  digitalWrite(DECK_GPIO_IO4, HIGH);
  DEBUG_PRINT("VL11: CS pins initialized (IO3=HIGH, IO4=HIGH)\n");
  
  // SPI の初期化
  spiBegin();
  DEBUG_PRINT("VL11: SPI initialized\n");
  
  // テストループ
  for (int ver3_i = 0; ver3_i < 300; ver3_i++) {
    // VL53L8CX のレジスタ 0x7FFF に 0x00 を書き込むテスト
    // SPI プロトコル: [16bit Address | 8bit Data]
    // Write bit = MSB of address byte set to 1 (0x80 | high_byte)
    
    uint16_t test_addr = 0x7FFF;
    uint8_t test_data = 0x00;
    
    // SPI トランザクション開始 (2MHz, ~1.3MHz actual)
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    
    // CS0 (IO3) を LOW にしてセンサー選択
    digitalWrite(DECK_GPIO_IO3, LOW);
    
    // アドレスの上位バイトを送信 (write bit を含む: 0x80 | high_byte)
    uint8_t addr_high = (uint8_t)((test_addr >> 8) | 0x80);
    uint8_t addr_low = (uint8_t)(test_addr & 0xFF);
    uint8_t dummy_rx = 0;
    
    spiExchange(1, &addr_high, &dummy_rx);
    spiExchange(1, &addr_low, &dummy_rx);
    spiExchange(1, &test_data, &dummy_rx);
    
    // CS0 を HIGH に戻してセンサーの測定終了
    digitalWrite(DECK_GPIO_IO3, HIGH);
    
    // SPI トランザクション終了
    spiEndTransaction();
    
    // 20ms 待機
    vTaskDelay(pdMS_TO_TICKS(20));
    
    /* sample heap every 8 iterations to avoid too much log spam */
    if ((ver3_i & 7) == 0) {
      DEBUG_PRINT("loop i=%d (addr=0x%04X, data=0x%02X, CS=IO3)\n", 
                  ver3_i, test_addr, test_data);
      heapSnap("during-loop");
      DEBUG_PRINT("\n");//ここでSPI通信で取得したデータを表示する
      DEBUG_PRINT("\n");
    }
  }
  
  DEBUG_PRINT("VL11: Low-level SPI test completed\n");
  /* snapshot heap after loop */
  heapSnap("after-loop");
  DEBUG_PRINT("WrByte end!!!!!!!!!!!!\n\n\n");
}

/* ===== Robust blob address snoop (FLASH vs RAM) ===== */
static void printBlobAddresses(void) {
  uintptr_t aFW    = (uintptr_t)(&VL53L8CX_FIRMWARE[0]);
  uintptr_t aCFG   = (uintptr_t)(&VL53L8CX_DEFAULT_CONFIGURATION[0]);
  uintptr_t aXTALK = (uintptr_t)(&VL53L8CX_DEFAULT_XTALK[0]);
  uintptr_t aNVM   = (uintptr_t)(&VL53L8CX_GET_NVM_CMD[0]);

  DEBUG_PRINT("VL8CX blob addrs: FW=0x%08" PRIxPTR " CFG=0x%08" PRIxPTR " XTALK=0x%08" PRIxPTR " NVM=0x%08" PRIxPTR "\n",
              aFW, aCFG, aXTALK, aNVM);
  DEBUG_PRINT("Note: FLASH ~0x080xxxxx, SRAM ~0x200xxxxx. Arrays must be in FLASH.\n");

  /* Consider them OK if they are non-zero and look like FLASH */
  int ok = (aFW && aCFG && aXTALK && aNVM);
  int inFlash =
      ((aFW & 0xFF000000u) == 0x08000000u) &&
      ((aCFG & 0xFF000000u) == 0x08000000u) &&
      ((aXTALK & 0xFF000000u) == 0x08000000u) &&
      ((aNVM & 0xFF000000u) == 0x08000000u);
  g_blobsOk = (ok && inFlash) ? 1 : 0;
  DEBUG_PRINT("VL8CX blobs %s, placement=%s\n", g_blobsOk ? "OK" : "BAD",
              inFlash ? "FLASH" : (ok ? "NOT-FLASH" : "MISSING"));
}

/* ===== Params / Logs ===== */
PARAM_GROUP_START(vl11)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, enable,  &g_running, onEnableUpdated)
  PARAM_ADD(PARAM_UINT8, rate_hz, &g_rate_hz)
  PARAM_ADD(PARAM_UINT8, testGen, &g_testGen)
  PARAM_ADD(PARAM_UINT8, heapsnap, &g_heapSnap)  /* write 1 to print heap */
  PARAM_ADD(PARAM_UINT8, blobs_ok, &g_blobsOk)   /* read-only diagnostic */
PARAM_GROUP_STOP(vl11)

LOG_GROUP_START(vl11)
  LOG_ADD(LOG_UINT32, tick, &g_tick)
  LOG_ADD(LOG_UINT16, s0,  &g_ranges_mm[0])
  // LOG_ADD(LOG_UINT16, s1,  &g_ranges_mm[1])
  // LOG_ADD(LOG_UINT16, s2,  &g_ranges_mm[2])
  // LOG_ADD(LOG_UINT16, s3,  &g_ranges_mm[3])
  // LOG_ADD(LOG_UINT16, s4,  &g_ranges_mm[4])
  // LOG_ADD(LOG_UINT16, s5,  &g_ranges_mm[5])
  // LOG_ADD(LOG_UINT16, s6,  &g_ranges_mm[6])
  // LOG_ADD(LOG_UINT16, s7,  &g_ranges_mm[7])
  // LOG_ADD(LOG_UINT16, s8,  &g_ranges_mm[8])
  // LOG_ADD(LOG_UINT16, s9,  &g_ranges_mm[9])
  // LOG_ADD(LOG_UINT16, s10, &g_ranges_mm[10])
LOG_GROUP_STOP(vl11)

/* ===== Deck glue ===== */
static void vl11Init(DeckInfo* info) {
  (void)info;
  heapSnap("init");
  printBlobAddresses();
}

bool vl11IsRunning(void) { return g_running != 0; }
uint16_t vl11GetLastMm(int index) { if (index < 0 || index >= VL11_NUM_SENSORS) return 0; return g_ranges_mm[index]; }

static const DeckDriver bcVL53L8CX11 = { .name = "bcVL53L8CX11", .init = vl11Init };
DECK_DRIVER(bcVL53L8CX11);
