
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

#include "deck.h"
#include "log.h"
#include "param.h"

#include "vl53l8cx11.h"
#include "vl53l8cx_api.h"
#include "platform.h"           /* VL53L8CX_Platform */

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
#include "vl53l8cx_main_cpp.h"


/* ===== Provide your REAL CS pins here ===== */
// const deckPin_t g_vl11_cs[VL11_NUM_SENSORS] = {
//   (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0},
//   (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0},
// };

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

#ifdef VL11_USE_CCM
__attribute__((section(".ccmram")))
#endif

/* public last distances */
static uint16_t g_ranges_mm[VL11_NUM_SENSORS];

/* ===== Heap probe helper ===== */
static inline void heapSnap(const char* tag) {
  size_t cur = xPortGetFreeHeapSize();
  size_t min = xPortGetMinimumEverFreeHeapSize();
  DEBUG_PRINT("HEAP[%s] cur=%u minEver=%u\n", (tag?tag:""), (unsigned)cur, (unsigned)min);
}



/* ===== worker ===== */
static void vl11Task(void* arg) {
  (void)arg;
  DEBUG_PRINT("vl11Task: Started (ver2 - minimal driver)\n");
  vl53l8cx_main();
}

/* Task handle (created on enable toggle) */
static TaskHandle_t g_task = NULL;

static void onEnableUpdated() {
  heapSnap(g_running ? "enable=1" : "enable=0");
  // if (g_running == NULL) {
  if (g_running && g_task == NULL) {
    xTaskCreate(vl11Task, "vl11", 384, NULL, tskIDLE_PRIORITY + 1, &g_task);
  }
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
  LOG_ADD(LOG_UINT16, s1,  &g_ranges_mm[1])
  LOG_ADD(LOG_UINT16, s2,  &g_ranges_mm[2])
  LOG_ADD(LOG_UINT16, s3,  &g_ranges_mm[3])
  LOG_ADD(LOG_UINT16, s4,  &g_ranges_mm[4])
  LOG_ADD(LOG_UINT16, s5,  &g_ranges_mm[5])
  LOG_ADD(LOG_UINT16, s6,  &g_ranges_mm[6])
  LOG_ADD(LOG_UINT16, s7,  &g_ranges_mm[7])
  LOG_ADD(LOG_UINT16, s8,  &g_ranges_mm[8])
  LOG_ADD(LOG_UINT16, s9,  &g_ranges_mm[9])
  LOG_ADD(LOG_UINT16, s10, &g_ranges_mm[10])
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
