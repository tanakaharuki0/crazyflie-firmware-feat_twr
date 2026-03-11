
#include "vl53l8cx11_ver4.h"

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "led.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"
#include "system.h"
#include "task.h"
#include "vl53l8cx_api.h"
/* Deck SPI and GPIO APIs */
#include "deck_constants.h"
#include "deck_digital.h"
#include "deck_spi.h"
#include "stm32f4xx_spi.h" /* For SPI_BaudRatePrescaler_* definitions */

/* ===== Memory/RAM strategy toggles =====
 * vl53l8cx_USE_CCM: place large non-DMA buffers in CCM (64KB fast SRAM, non-DMA) to free main SRAM.
 * vl53l8cx_DEFAULT_RATE_HZ: per-sensor ranging frequency (before multiplexing).
 */
#ifndef vl53l8cx_USE_CCM
#define vl53l8cx_USE_CCM 1
#endif
#ifndef vl53l8cx_DEFAULT_RATE_HZ
#define vl53l8cx_DEFAULT_RATE_HZ 10
#endif
// #include "vl53l8cx_arena.h"

/* Extern-only declarations of the ULD blobs. Must be defined exactly once elsewhere. */
#include "../../platform/interface/platform_vl53l8cx.h"
#include "vl53l8cx_buffers.h"

#define DECK_SPI_MODE3

/* ===== Provide your REAL CS pins here ===== */
const deckPin_t g_vl53l8cx_cs[vl53l8cx_NUM_SENSORS] = {
    (deckPin_t){0},
};

/* ===== Driver state (ultra-low RAM) ===== */
static volatile uint8_t g_running = 0;
static uint8_t g_rate_hz = vl53l8cx_DEFAULT_RATE_HZ;
static uint8_t g_testGen = 1;
// static uint8_t  g_inited  = 0;
static uint32_t g_tick = 0;

/* Heap snap + diagnostics */
static uint8_t g_heapSnap = 0;
static uint8_t g_blobsOk = 0; /* 1 if blobs look valid & in flash */
static TaskHandle_t g_task = NULL;
// static uint8_t  g_initOk[vl53l8cx_NUM_SENSORS] = {0};

/* Build-time switch to bypass ULD init for isolation tests (0 = skip, 1 = call init) */
#ifndef vl53l8cx_CALL_INIT
#define vl53l8cx_CALL_INIT 1
#endif

/* ONE shared configuration + ONE shared results */
// NO_DMA_CCM_SAFE_ZERO_INIT static VL53L8CX_Configuration g_dev;
#ifdef vl53l8cx_USE_CCM
__attribute__((section(".ccmram")))
#endif
// static VL53L8CX_ResultsData g_res;

/* public last distances */
static uint16_t g_ranges_mm[vl53l8cx_NUM_SENSORS];

// #ifdef vl53l8cx_USE_CCM
// __attribute__((section(".ccmram")))
// #endif
VL53L8CX_Configuration MDev[vl53l8cx_NUM_SENSORS];
VL53L8CX_ResultsData Results;

uint8_t callbacked = 0;
VL53L8CX_Configuration Dev;  // Sensor configuration
void Ranging_Basic(uint16_t DevAddr)
{
    uint8_t status, loop, isAlive;
    // uint isReady, i;

    Dev.platform.address = DevAddr;

    // (Optional) Check if there is a VL53L8CX sensor connected
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive || status)
    {
        DEBUG_PRINT("VL53L8CX not detected at requested address\n");
        return;
    }
    // DEBUG_PRINT("alive\n");
    // (Mandatory) Init VL53L8CX sensor
    status = vl53l8cx_init(&Dev);
    if (status)
    {
        DEBUG_PRINT("VL53L8CX ULD Loading failed\n");
        return;
    }
    DEBUG_PRINT("ULD ready\n");

    // Ranging loop
    status = vl53l8cx_set_ranging_frequency_hz(&Dev, 30);
    if (status)
    {
        DEBUG_PRINT("set_ranging_frequency_hz failed, status %u\n", status);
        return;
    }
    status = vl53l8cx_start_ranging(&Dev);
    loop = 0;
    while (loop < 30000)
    {
        // status = vl53l8cx_check_data_ready(&Dev, &isReady);
        // if (isReady)
        // {
        //     vl53l8cx_get_ranging_data(&Dev, &Results);
        //     DEBUG_PRINT("Print data no : %3u\n", Dev.streamcount);
        //     for (i = 0; i < 16; i++)
        //     {
        //         DEBUG_PRINT("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
        //                     Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
        //                     Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
        //     }
        //     DEBUG_PRINT("\n");
        //     loop++;
        // }
        DEBUG_PRINT("l\n");
        VL53L8CX_WaitMs(&(Dev.platform), 50);
    }
}

int Init_Sensor(uint16_t DevAddr, uint8_t Frequency)
{
    uint8_t status, isAlive;

    MDev[DevAddr].platform.address = DevAddr;

    status = vl53l8cx_is_alive(&MDev[DevAddr], &isAlive);
    if (!isAlive || status)
    {
        DEBUG_PRINT("vl53l8cx_is_alive failed, status %u[%d]\n", status, DevAddr);
        return (0);
    }
    status = vl53l8cx_init(&MDev[DevAddr]);
    if (status)
    {
        DEBUG_PRINT("VL53L8CX ULD Loading failed_init[%d]. status is %u\n", DevAddr, status);
        return (0);
    }
    // DEBUG_PRINT("VL53L8CX ULD ready ! (Version : %s)[%d]\n", VL53L8CX_API_REVISION, DevAddr);

    status = vl53l8cx_set_ranging_frequency_hz(&MDev[DevAddr], Frequency);
    if (status)
    {
        DEBUG_PRINT("vl53l8cx_set_ranging_frequency_hz failed, status %u[%d]\n", status, DevAddr);
        return (0);
    }
    status = vl53l8cx_start_ranging(&MDev[DevAddr]);
    if (status)
    {
        DEBUG_PRINT("vl53l8cx_start_ranging failed, status %u[%d]\n", status, DevAddr);
        return (0);
    }
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
    // size_t cur = xPortGetFreeHeapSize();
    // size_t min = xPortGetMinimumEverFreeHeapSize();
    // DEBUG_PRINT("HEAP%s cur%u min%u\n", (tag ? tag : ""), (unsigned)cur, (unsigned)min);
}

void led_debug(int seconds)
{
    int frequency = 10;
    for (int i = 0; i < seconds * frequency; i++)
    {
        ledSet(LED_GREEN_R, true);
        vTaskDelay(pdMS_TO_TICKS(1000 / frequency));
        ledSet(LED_GREEN_R, false);
        vTaskDelay(pdMS_TO_TICKS(1000 / frequency));
    }
}

/* ===== CS helpers ===== */
static inline void cs_low(int i) { vl53l8cx_GPIO_WRITE(g_vl53l8cx_cs[i], LOW); }
static inline void cs_high(int i) { vl53l8cx_GPIO_WRITE(g_vl53l8cx_cs[i], HIGH); }

static void vl53l8cxTask(void* arg)
{
    (void)arg;
    systemWaitStart();

    // int tmp_sensor = 0;

    for (;;)
    {
        // DEBUG_PRINT("w\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        // DEBUG_PRINT("w\n");
        if (callbacked)
        {
            // DEBUG_PRINT("b\n");
            led_debug(2);
            break;
        }
    }

    // int n = 0;
    // uint8_t status;
    init_IO();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    Ranging_Basic(0);
    // このループは、11個のセンサーがすべて初期化できるまで繰り返す
    // while (1)
    // {
    //     if (Init_Sensor(tmp_sensor, 30))
    //     {
    //         // DEBUG_PRINT("Init_Sensor %d end!!!!!!!!!!!!\n\n", n);
    //         n++;
    //         if (n >= vl53l8cx_NUM_SENSORS)
    //         {
    //             break;
    //         }
    //     }
    //     // vTaskDelay_for_spi_pause(pdMS_TO_TICKS(500));
    //     VL53L8CX_WaitMs_spi_pause(&MDev[tmp_sensor].platform, 500);
    // }

    // led_debug(2);

    // uint8_t isReady = 0;
    // int loop = 0;
    // while (loop < 10)
    // {
    //     status = vl53l8cx_check_data_ready(&MDev[tmp_sensor], &isReady);
    //     if (isReady)
    //     {
    //         vl53l8cx_get_ranging_data(&MDev[tmp_sensor], &Results);
    //         DEBUG_PRINT("Print data no : %3u\n", MDev[tmp_sensor].streamcount);
    //         for (int i = 0; i < 16; i++)
    //         {
    //             DEBUG_PRINT("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
    //                         Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
    //                         Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
    //         }
    //         DEBUG_PRINT("\n");
    //         loop++;
    //     }
    //     VL53L8CX_WaitMs_spi_pause(&(MDev[tmp_sensor].platform), 5);
    // }
    spiEndTransaction();
    vTaskDelete(NULL);
}

static void onEnableUpdated()
{
    // DEBUG_PRINT("onE %d\n", g_running);
    // heapSnap(g_running ? "ena1" : "ena0");
    callbacked = 1;
    // DEBUG_PRINT("callbacked\n");

    // Print all tasks
    // {
    // char taskBuffer[100];
    // vTaskList(taskBuffer);
    // DEBUG_PRINT("\n");
    // DEBUG_PRINT("%s", taskBuffer);
    // }

    // if (g_running && g_task == NULL)
    // {
    //     xTaskCreate(vl53l8cxTask, "vl53l8cx", 384, NULL, tskIDLE_PRIORITY + 3, &g_task);
    // }

    // memset(&g_dev, 0, sizeof(g_dev));
    // g_dev.platform.address = (uint16_t)0;

    // int n = 0;
    // uint8_t status;
    // init_IO();
    // spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    // // DEBUG_PRINT("Init_IO() success \n\n");
    // // このループは、11個のセンサーがすべて初期化できるまで繰り返す
    // // Init_Sensor(0, 1);
    // while (1)
    // {
    //     if (Init_Sensor(n, 30))
    //     {
    //         // DEBUG_PRINT("Init_Sensor %d end!!!!!!!!!!!!\n\n", n);
    //         n++;
    //         if (n >= vl53l8cx_NUM_SENSORS)
    //         {
    //             break;
    //         }
    //     }
    //     // vTaskDelay_for_spi_pause(pdMS_TO_TICKS(500));
    //     VL53L8CX_WaitMs_spi_pause(&MDev[0].platform, 500);
    // }

    // uint8_t isReady = 0;
    // int loop = 0;
    // while (loop < 10)
    // {
    //     status = vl53l8cx_check_data_ready(&MDev[0], &isReady);
    //     if (isReady)
    //     {
    //         vl53l8cx_get_ranging_data(&MDev[0], &Results);
    //         DEBUG_PRINT("Print data no : %3u\n", MDev[0].streamcount);
    //         for (int i = 0; i < 16; i++)
    //         {
    //             DEBUG_PRINT("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
    //                         Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
    //                         Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
    //         }
    //         DEBUG_PRINT("\n");
    //         loop++;
    //     }
    //     VL53L8CX_WaitMs_spi_pause(&(MDev[0].platform), 5);
    // }
    // spiEndTransaction();
}

/* ===== Robust blob address snoop (FLASH vs RAM) ===== */
static void printBlobAddresses(void)
{
    if (g_task == NULL)
    // if (g_running && g_task == NULL)
    {
        BaseType_t rc = xTaskCreate(vl53l8cxTask, "vl53l8cx", 384, NULL, tskIDLE_PRIORITY + 4, &g_task);
        if (rc == pdPASS)
        {
            DEBUG_PRINT("OK\n");
        }
        else
        {
            DEBUG_PRINT("FAIL\n");
        }
    }
    // uintptr_t aFW = (uintptr_t)(&VL53L8CX_FIRMWARE[0]);
    // uintptr_t aCFG = (uintptr_t)(&VL53L8CX_DEFAULT_CONFIGURATION[0]);
    // uintptr_t aXTALK = (uintptr_t)(&VL53L8CX_DEFAULT_XTALK[0]);
    // uintptr_t aNVM = (uintptr_t)(&VL53L8CX_GET_NVM_CMD[0]);

    // DEBUG_PRINT("VL8CX blob addrs: FW=0x%08" PRIxPTR " CFG=0x%08" PRIxPTR " XTALK=0x%08" PRIxPTR " NVM=0x%08" PRIxPTR
    //             "\n",
    //             aFW, aCFG, aXTALK, aNVM);
    // DEBUG_PRINT("Note: FLASH ~0x080xxxxx, SRAM ~0x200xxxxx. Arrays must be in FLASH.\n");

    /* Consider them OK if they are non-zero and look like FLASH */
    // int ok = (aFW && aCFG && aXTALK && aNVM);
    // int inFlash = ((aFW & 0xFF000000u) == 0x08000000u) && ((aCFG & 0xFF000000u) == 0x08000000u) &&
    //               ((aXTALK & 0xFF000000u) == 0x08000000u) && ((aNVM & 0xFF000000u) == 0x08000000u);
    // g_blobsOk = (ok && inFlash) ? 1 : 0;
    // DEBUG_PRINT("VL8CX blobs %s, placement=%s\n", g_blobsOk ? "OK" : "BAD",
    //             inFlash ? "FLASH" : (ok ? "NOT-FLASH" : "MISSING"));
}

/* ===== Params / Logs ===== */
PARAM_GROUP_START(vl53l8cx)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, enable, &g_running, onEnableUpdated)
PARAM_ADD(PARAM_UINT8, rate_hz, &g_rate_hz)
PARAM_ADD(PARAM_UINT8, testGen, &g_testGen)
PARAM_ADD(PARAM_UINT8, heapsnap, &g_heapSnap) /* write 1 to print heap */
PARAM_ADD(PARAM_UINT8, blobs_ok, &g_blobsOk)  /* read-only diagnostic */
PARAM_GROUP_STOP(vl53l8cx)

LOG_GROUP_START(vl53l8cx)
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
LOG_GROUP_STOP(vl53l8cx)

/* ===== Deck glue ===== */
static void vl53l8cxInit(DeckInfo* info)
{
    (void)info;
    heapSnap("init");
    printBlobAddresses();
}

bool vl53l8cxIsRunning(void) { return g_running != 0; }
// uint16_t vl53l8cxGetLastMm(int index) { if (index < 0 || index >= vl53l8cx_NUM_SENSORS) return 0; return
// g_ranges_mm[index];
// }

static const DeckDriver bcVL53L8CX11 = {.name = "bcVL53L8CX11", .init = vl53l8cxInit};
DECK_DRIVER(bcVL53L8CX11);
