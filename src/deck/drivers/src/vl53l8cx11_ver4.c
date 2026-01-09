
#include "vl53l8cx11_ver4.h"

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"
#include "task.h"
#include "vl53l8cx_api.h"
/* Deck SPI and GPIO APIs */
#include "deck_constants.h"
#include "deck_digital.h"
#include "deck_spi.h"
#include "stm32f4xx_spi.h" /* For SPI_BaudRatePrescaler_* definitions */

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
#include "../../platform/interface/platform_vl53l8cx.h"
#include "vl53l8cx_buffers.h"

/* ===== Provide your REAL CS pins here ===== */
const deckPin_t g_vl11_cs[VL11_NUM_SENSORS] = {
    (deckPin_t){0},
};

/* ===== Driver state (ultra-low RAM) ===== */
static volatile uint8_t g_running = 0;
static uint8_t g_rate_hz = VL11_DEFAULT_RATE_HZ;
static uint8_t g_testGen = 1;
// static uint8_t  g_inited  = 0;
static uint32_t g_tick = 0;

/* Heap snap + diagnostics */
static uint8_t g_heapSnap = 0;
static uint8_t g_blobsOk = 0; /* 1 if blobs look valid & in flash */
// static TaskHandle_t g_task = NULL;
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

// #ifdef VL11_USE_CCM
// __attribute__((section(".ccmram")))
// #endif
VL53L8CX_Configuration MDev[VL11_NUM_SENSORS];
VL53L8CX_ResultsData Results;

int Init_Sensor(uint16_t DevAddr, uint8_t Frequency)
{
    uint8_t status, isAlive;

    MDev[DevAddr].platform.address = DevAddr;

    status = vl53l8cx_is_alive(&MDev[DevAddr], &isAlive);
    if (status)
    {
        DEBUG_PRINT("vl53l8cx_is_alive failed, status %u[%d]\n", status, DevAddr);
        return (0);
    }
    status = vl53l8cx_init(&MDev[DevAddr]);
    if (status)
    {
        // DEBUG_PRINT("VL53L8CX ULD Loading failed_init[%d]. status is %u\n", DevAddr, status);
        return (0);
    }
    DEBUG_PRINT("VL53L8CX ULD ready ! (Version : %s)[%d]\n", VL53L8CX_API_REVISION, DevAddr);

    status = vl53l8cx_set_ranging_frequency_hz(&MDev[DevAddr], Frequency);
    // if(status) {
    // 	DEBUG_PRINT("vl53l8cx_set_ranging_frequency_hz failed, status %u[%d]\n", status, DevAddr);
    // 	return(0);
    // }
    // status = vl53l8cx_start_ranging(&MDev[DevAddr]);
    return (1);
}

void Start_Ranging(uint16_t DevAddr)
{
    uint8_t status;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_start_ranging(&MDev[DevAddr]);
}

uint8_t DevAddr[11];
uint8_t ReStart[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t NumRdy[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// この関数は、すべてのセンサーから距離測定データを取得し、表示する
// さらに、データが取得できなかったセンサーについては、再起動を試みる
// この関数の中で呼ばれる関数は以下の通り
// vl53l8cx_get_ranging_data() : 距離測定データの取得
// Start_Ranging() : 距離測定の開始／再起動
void Gget_Ranging()
{
    // uint8_t status, loop, isAlive, isReady;
    char i;
    int k;

    for (k = 0; k < 11; k++) DevAddr[k] = 0xFF;
    k = Ser_IT();  // In The platform.cpp
    DEBUG_PRINT("Ser_IT returned: 0x%04X\n", k);

    if (k == 0)
    {
        return;
    }

    if ((k & 0x0020) != 0) DevAddr[10] = 10;
    if ((k & 0x0040) != 0) DevAddr[9] = 9;
    if ((k & 0x0080) != 0) DevAddr[8] = 8;
    if ((k & 0x0100) != 0) DevAddr[7] = 7;
    if ((k & 0x0200) != 0) DevAddr[6] = 6;
    if ((k & 0x0400) != 0) DevAddr[5] = 5;
    if ((k & 0x0800) != 0) DevAddr[4] = 4;
    if ((k & 0x1000) != 0) DevAddr[3] = 3;
    if ((k & 0x2000) != 0) DevAddr[2] = 2;
    if ((k & 0x4000) != 0) DevAddr[1] = 1;
    if ((k & 0x8000) != 0) DevAddr[0] = 0;
    for (k = 0; k < 11; k++)
    {
        if (DevAddr[k] != 0xFF)
        {
            MDev[DevAddr[k]].platform.address = DevAddr[k];
            vl53l8cx_get_ranging_data(&MDev[DevAddr[k]], &Results);
            // DEBUG_PRINT("[%d]Print data no : %3u\n", DevAddr[k], MDev[DevAddr[k]].streamcount);
            DEBUG_PRINT("[%2d] ", DevAddr[k]);
            // Resultsの内容をすべて表示する
            DEBUG_PRINT("T=%dC, nb0=%u, d0=%d, st0=%u\n", (int)Results.silicon_temp_degc,
                        (unsigned)Results.nb_target_detected[0], (int)Results.distance_mm[0],
                        (unsigned)Results.target_status[0]);
            for (i = 0; i < 16; i++)
            {
                if (Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i] == 5)
                {
                    DEBUG_PRINT("[%4d]", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
                }
                else
                {
                    DEBUG_PRINT("--%02X--", Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i]);
                }
            }
            DEBUG_PRINT(" R[%3d]", NumRdy[k]);
            DEBUG_PRINT(" S[%d]", ReStart[k]);
            DEBUG_PRINT("\n");
            NumRdy[k] = 0;
        }
        else
        {
            NumRdy[k]++;
        }
    }
    for (k = 0; k < 11; k++)
    {
        if (NumRdy[k] > 250)
        {
            DEBUG_PRINT("Restart DevAddr[%d] NumRdy[%d]\n", k, NumRdy[k]);
            Start_Ranging(k);
            ReStart[k]++;
        }
    }
}

/* ===== Heap probe helper ===== */
static inline void heapSnap(const char* tag)
{
    size_t cur = xPortGetFreeHeapSize();
    size_t min = xPortGetMinimumEverFreeHeapSize();
    DEBUG_PRINT("HEAP[%s] cur=%u minEver=%u\n", (tag ? tag : ""), (unsigned)cur, (unsigned)min);
}

/* ===== CS helpers ===== */
static inline void cs_low(int i) { VL11_GPIO_WRITE(g_vl11_cs[i], LOW); }
static inline void cs_high(int i) { VL11_GPIO_WRITE(g_vl11_cs[i], HIGH); }

// static void vl11Task(void* arg) {
//   (void)arg;
//   DEBUG_PRINT("vl11Task: Started (ver2 - minimal driver)\n");
// }

static void onEnableUpdated()
{
    // heapSnap(g_running ? "enable=1" : "enable=0");
    // if (g_running && g_task == NULL) {
    //   xTaskCreate(vl11Task, "vl11", 384, NULL, tskIDLE_PRIORITY + 1, &g_task);
    // }

    memset(&g_dev, 0, sizeof(g_dev));
    g_dev.platform.address = (uint16_t)0;

    // if (g_blobsOk){
    //   vl11_arena_reset(12*1024);   // start with 12 KB; adjust if you see OOM log
    //   int8_t st = vl53l8cx_init(&g_dev);
    //   DEBUG_PRINT("vl8cx init -> %d\n", (int)st);
    //   if (st == VL53L8CX_STATUS_OK) {
    //     /* Force 4x4 to shrink result buffers */
    //     vl53l8cx_set_resolution(&g_dev, VL53L8CX_RESOLUTION_4X4);
    //     /* If your ULD exposes it, keep 1 target/zone (API name differs across drops)
    //        Examples:
    //        // vl53l8cx_set_nb_target_per_zone(&g_dev, 1);
    //        // vl53l8cx_set_nb_targets_per_zone(&g_dev, 1);
    //     */
    //     vl53l8cx_set_ranging_frequency_hz(&g_dev, g_rate_hz);
    //     (void)vl53l8cx_start_ranging(&g_dev);
    // }else {
    //     DEBUG_PRINT("vl8cx skipped init (blobs missing)\n");
    //   }
    // }
    // uint8_t status = VL53L8CX_STATUS_OK;
    // DEBUG_PRINT("\n\n\nWrByte start!!!!!!!!!!!!\n");
    // /* snapshot heap before heavy loop */
    // heapSnap("before-loop");
    // // 低レベルSPI通信テスト: CS pin と SPI 直接制御
    // DEBUG_PRINT("VL11: Low-level SPI test starting...\n");

    // // CS pins の初期化 (IO3, IO4 を OUTPUT mode にして HIGH に設定)
    // pinMode(DECK_GPIO_IO3, OUTPUT);
    // pinMode(DECK_GPIO_IO4, OUTPUT);
    // digitalWrite(DECK_GPIO_IO3, HIGH);
    // digitalWrite(DECK_GPIO_IO4, HIGH);
    // DEBUG_PRINT("VL11: CS pins initialized (IO3=HIGH, IO4=HIGH)\n");
    int n = 0;
    uint8_t status;
    init_IO();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    // DEBUG_PRINT("Init_IO() success \n\n");
    // このループは、11個のセンサーがすべて初期化できるまで繰り返す
    // Init_Sensor(0, 1);
    while (1)
    {
        if (Init_Sensor(n, 1) == 0)
        {
            DEBUG_PRINT("Init_Sensor %d end!!!!!!!!!!!!\n\n", n);
            n++;
            if (n >= VL11_NUM_SENSORS)
            {
                break;
            }
        }
        vTaskDelay_for_spi_pause(pdMS_TO_TICKS(500));
    }

    for (int i = 0; i < VL11_NUM_SENSORS; i++)
    {
        status = vl53l8cx_start_ranging(&MDev[i]);
        DEBUG_PRINT("vl53l8cx_start_ranging %d status=%d\n", i, status);
    }

    vTaskDelay_for_spi_pause(pdMS_TO_TICKS(100));
    Sel_Dev(MDev[0].platform.address);
    uint8_t isReady = 0;
    vl53l8cx_check_data_ready(&MDev[0], &isReady);
    DEBUG_PRINT("vl53l8cx_check_data_ready %d isReady=%d\n", MDev[0].platform.address, isReady);
    while (true)
    {
        Gget_Ranging();
        // if (isReady)
        // {
        //     // 測定結果格納用バッファ
        //     VL53L8CX_ResultsData results = {0};

        //     // 距離データ取得
        //     uint8_t status = vl53l8cx_get_ranging_data(&MDev[i], &results);

        //     if (status == VL53L8CX_STATUS_OK)
        //     {
        //         // 4x4解像度なので16個のデータがある
        //         DEBUG_PRINT("Sensor[%d] distances(mm): ", i);
        //         for (int j = 0; j < 16; j++)
        //         {
        //             DEBUG_PRINT("%d ", results.distance_mm[j]);
        //         }
        //         DEBUG_PRINT("\n");

        //         // または最初の距離だけを表示
        //         g_ranges_mm[i] = results.distance_mm[0];
        //         DEBUG_PRINT("Sensor[%d] first distance: %d mm\n", i, results.distance_mm[0]);
        //     }
        // }

        vTaskDelay_for_spi_pause(pdMS_TO_TICKS(100));  // 100ms間隔で取得
    }
    spiEndTransaction();
}

/* ===== Robust blob address snoop (FLASH vs RAM) ===== */
static void printBlobAddresses(void)
{
    uintptr_t aFW = (uintptr_t)(&VL53L8CX_FIRMWARE[0]);
    uintptr_t aCFG = (uintptr_t)(&VL53L8CX_DEFAULT_CONFIGURATION[0]);
    uintptr_t aXTALK = (uintptr_t)(&VL53L8CX_DEFAULT_XTALK[0]);
    uintptr_t aNVM = (uintptr_t)(&VL53L8CX_GET_NVM_CMD[0]);

    DEBUG_PRINT("VL8CX blob addrs: FW=0x%08" PRIxPTR " CFG=0x%08" PRIxPTR " XTALK=0x%08" PRIxPTR " NVM=0x%08" PRIxPTR
                "\n",
                aFW, aCFG, aXTALK, aNVM);
    DEBUG_PRINT("Note: FLASH ~0x080xxxxx, SRAM ~0x200xxxxx. Arrays must be in FLASH.\n");

    /* Consider them OK if they are non-zero and look like FLASH */
    int ok = (aFW && aCFG && aXTALK && aNVM);
    int inFlash = ((aFW & 0xFF000000u) == 0x08000000u) && ((aCFG & 0xFF000000u) == 0x08000000u) &&
                  ((aXTALK & 0xFF000000u) == 0x08000000u) && ((aNVM & 0xFF000000u) == 0x08000000u);
    g_blobsOk = (ok && inFlash) ? 1 : 0;
    DEBUG_PRINT("VL8CX blobs %s, placement=%s\n", g_blobsOk ? "OK" : "BAD",
                inFlash ? "FLASH" : (ok ? "NOT-FLASH" : "MISSING"));
}

/* ===== Params / Logs ===== */
PARAM_GROUP_START(vl11)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, enable, &g_running, onEnableUpdated)
PARAM_ADD(PARAM_UINT8, rate_hz, &g_rate_hz)
PARAM_ADD(PARAM_UINT8, testGen, &g_testGen)
PARAM_ADD(PARAM_UINT8, heapsnap, &g_heapSnap) /* write 1 to print heap */
PARAM_ADD(PARAM_UINT8, blobs_ok, &g_blobsOk)  /* read-only diagnostic */
PARAM_GROUP_STOP(vl11)

LOG_GROUP_START(vl11)
LOG_ADD(LOG_UINT32, tick, &g_tick)
LOG_ADD(LOG_UINT16, s0, &g_ranges_mm[0])
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
static void vl11Init(DeckInfo* info)
{
    (void)info;
    heapSnap("init");
    printBlobAddresses();
}

bool vl11IsRunning(void) { return g_running != 0; }
// uint16_t vl11GetLastMm(int index) { if (index < 0 || index >= VL11_NUM_SENSORS) return 0; return g_ranges_mm[index];
// }

static const DeckDriver bcVL53L8CX11 = {.name = "bcVL53L8CX11", .init = vl11Init};
DECK_DRIVER(bcVL53L8CX11);
