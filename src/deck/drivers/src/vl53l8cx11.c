
/*
 * 日本語コメント: インクルード群
 * - 標準ヘッダや FreeRTOS、デバッグ/デッキ API、VL53L8CX の API を読み込みます。
 * - ここで取り込むヘッダにより後続の関数（VL53L8CX_WrByte 等）や構造体が利用可能になります。
 */
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

// main
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "vl53l8cx_api.h"
#include "vl53l8cx_buffers.h"
#include "platform.h"
#include <stdio.h>

/* Only include POSIX headers and provide stdin-based helpers when compiling
 * for a host/desktop environment. When cross-compiling for the embedded
 * target these headers are not available, so instead provide RTOS-based
 * stubs above. We detect host builds using common host macros. */
#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>

/* Host / POSIX helpers */
static void sleep_ms(unsigned int ms) { usleep(ms * 1000u); }

static int serial_readable(void) {
  int n = 0;
  if (ioctl(STDIN_FILENO, FIONREAD, &n) == 0) {
    return n;
  }
  fd_set rf;
  FD_ZERO(&rf);
  FD_SET(STDIN_FILENO, &rf);
  struct timeval tv = {0, 0};
  int r = select(STDIN_FILENO + 1, &rf, NULL, NULL, &tv);
  return (r > 0) ? 1 : 0;
}

static int serial_read(char *buf, size_t size) {
  ssize_t r = read(STDIN_FILENO, buf, size);
  if (r < 0) return 0;
  return (int)r;
}
#else
#if defined(__GNUC__)
/* Mark as possibly-unused to avoid -Werror=unused-function in embedded builds */
static void sleep_ms(unsigned int ms) __attribute__((unused));
static int serial_readable(void) __attribute__((unused));
static int serial_read(char *buf, size_t size) __attribute__((unused));
#endif
/* Embedded/RTOS build: use vTaskDelay for sleeps and provide no-op serial
 * helpers. If an embedded interactive VCP is needed it should be provided
 * by platform code. */
static void sleep_ms(unsigned int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
static int serial_readable(void) { (void)0; return 0; }
static int serial_read(char *buf, size_t size) { (void)buf; (void)size; return 0; }
#endif
/* Host-only example state and helpers. Guarded so embedded firmware does
 * not pull in host test utilities (stdin, init_IO, Ser_IT, etc). */
#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
VL53L8CX_Configuration		Dev;

VL53L8CX_ResultsData 	Results;		// Results data from VL53L8CX 

//---------------------------------------------------------------------------
//  Serial_vcp Input
//---------------------------------------------------------------------------
char    ucmd[64];
int     ucmd_p = 0;
// この関数は platform.cpp の Ser_IT() から呼ばれる
// 改行コードが来るまで受け付け、バッファに貯める
// 改行コードが来たら、バッファを0終端して、0を返す
int get_Vcp()
{
    int     n, k;
    char    buf[64];

  k = serial_readable();
  if(k == 0) return(0);
  k = serial_read(buf, sizeof(buf));
    if(k != 0) {
        for(n = 0; n < k; n++) {
            if(buf[n] == '\r' || buf[n] == '\n') {
                ucmd[ucmd_p] = 0;
                ucmd_p = 0;
                return(k);
            } else {
                ucmd[ucmd_p] = buf[n];
                if(ucmd_p < 64) ucmd_p++;
            }
        }
    }
    return(0);
}

//----------------------------------------------------------------
// Example_1_Ranging_Basic(The VL53L8CX ULD package)
//----------------------------------------------------------------
// この関数は VL53L8CX ULD の基本的な使い方を示す
// 1Hzで10回距離測定を行い、結果を表示する
// DevAddr はセンサーのI2Cアドレス
// platform.cpp の Sel_Dev() で使われる
//----------------------------------------------------------------
void Ranging_Basic(uint16_t DevAddr)
{
    uint8_t 				status, loop, isAlive, isReady, i;
	VL53L8CX_Configuration 	Dev;			// Sensor configuration 

    Dev.platform.address = DevAddr;

    // (Optional) Check if there is a VL53L8CX sensor connected
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status) {
		printf("VL53L8CX not detected at requested address\n");
		return;
	}

    // (Mandatory) Init VL53L8CX sensor
	status = vl53l8cx_init(&Dev);
	if(status) {
		printf("VL53L8CX ULD Loading failed\n");
		return;
	}
    printf("VL53L8CX ULD ready ! (Version : %s)\n", VL53L8CX_API_REVISION);

    // Ranging loop
    status = vl53l8cx_set_ranging_frequency_hz(&Dev, 1);
	if(status) {
		printf("vl53l8cx_set_ranging_frequency_hz failed, status %u\n", status);
		return;
	}
    status = vl53l8cx_start_ranging(&Dev);
    loop = 0;
	while(loop < 10) {
        status = vl53l8cx_check_data_ready(&Dev, &isReady);
        if(isReady) {
			vl53l8cx_get_ranging_data(&Dev, &Results);
            printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++) {
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
					Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
			loop++;
		}
        VL53L8CX_WaitMs(&(Dev.platform), 5);
    }
}

//----------------------------------------------------------------
// Multiple Sensor(by Kizaki)
// 
// vl53l8cx_is_alive() : VL53L8CX ULD API
// vl53l8cx_init() : VL53L8CX ULD API
// vl53l8cx_set_ranging_frequency_hz() : VL53L8CX ULD API
// vl53l8cx_start_ranging() : VL53L8CX ULD API
//----------------------------------------------------------------
VL53L8CX_Configuration 	MDev[11];

// DevAddr is 0 to 10
// この関数はセンサーの初期化を行う
// 
int Init_Sensor(uint16_t DevAddr, uint8_t Frequency)
{
    uint8_t     status, isAlive;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_is_alive(&MDev[DevAddr], &isAlive);
    if(status) {
		printf("VL53L8CX ULD Loading failed_alive[%d]\n", DevAddr);
		return(0);
	}
    status = vl53l8cx_init(&MDev[DevAddr]);
	if(status) {
		printf("VL53L8CX ULD Loading failed_init[%d]\n", DevAddr);
		return(0);
	}
    printf("VL53L8CX ULD ready ! (Version : %s)[%d]\n", VL53L8CX_API_REVISION, DevAddr);

    status = vl53l8cx_set_ranging_frequency_hz(&MDev[DevAddr], Frequency);
	if(status) {
		printf("vl53l8cx_set_ranging_frequency_hz failed, status %u[%d]\n", status, DevAddr);
		return(0);
	}
    //status = vl53l8cx_start_ranging(&MDev[DevAddr]);
    return(1);
}

// この関数はセンサーの距離測定を開始する
// DevAddr is 0 to 10
void Start_Ranging(uint16_t DevAddr)
{
    uint8_t     status;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_start_ranging(&MDev[DevAddr]);
}

uint8_t DevAddr[11];
uint8_t ReStart[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t NumRdy[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// この関数は、すべてのセンサーから距離測定データを取得し、表示する
// さらに、データが取得できなかったセンサーについては、再起動を試みる
// この関数の中で呼ばれる関数は以下の通り
// vl53l8cx_get_ranging_data() : 距離測定データの取得
// Start_Ranging() : 距離測定の開始／再起動
void Gget_Ranging()
{
  char        i;
  int         k;

    for(k = 0; k < 11; k++) DevAddr[k] = 0xFF;
    k = Ser_IT();       // In The platform.cpp
    if(k == 0) return;

    if((k & 0x0020) != 0) DevAddr[10] = 10;
    if((k & 0x0040) != 0) DevAddr[9] = 9;
    if((k & 0x0080) != 0) DevAddr[8] = 8;
    if((k & 0x0100) != 0) DevAddr[7] = 7;
    if((k & 0x0200) != 0) DevAddr[6] = 6;
    if((k & 0x0400) != 0) DevAddr[5] = 5;
    if((k & 0x0800) != 0) DevAddr[4] = 4;
    if((k & 0x1000) != 0) DevAddr[3] = 3;
    if((k & 0x2000) != 0) DevAddr[2] = 2;
    if((k & 0x4000) != 0) DevAddr[1] = 1;
    if((k & 0x8000) != 0) DevAddr[0] = 0;
    for(k = 0; k < 11; k++) {
        if(DevAddr[k] != 0xFF) {
            MDev[DevAddr[k]].platform.address = DevAddr[k];
            vl53l8cx_get_ranging_data(&MDev[DevAddr[k]], &Results);
            //printf("[%d]Print data no : %3u\n", DevAddr[k], MDev[DevAddr[k]].streamcount);
            printf("[%2d] ", DevAddr[k]);
			for(i = 0; i < 16; i++) {
                if(Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i] == 5) {
                    printf("[%4d]", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
                } else {
                    printf("--%02X--", Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i]);
                }
			}
            printf(" R[%3d]", NumRdy[k]);
            printf(" S[%d]", ReStart[k]);
			printf("\n");
            NumRdy[k] = 0;
        } else {
            NumRdy[k]++;
        }
    }
    for(k = 0; k < 11; k++) {
        if(NumRdy[k] > 250) {
            printf("Restart DevAddr[%d] NumRdy[%d]\n", k, NumRdy[k]);
            Start_Ranging(k);
            ReStart[k]++;
        }
    }
}

//----------------------------------------------------------------
// Main
// 
// init_IO() : In The platform.cpp
// Init_Sensor() : Multiple Sensor(by Kizaki)
// vl53l8cx_start_ranging() : VL53L8CX ULD API
// Gget_Ranging() : Multiple Sensor(by Kizaki)
// get_Vcp() : In The platform.cpp
// vl53l8cx_set_ranging_frequency_hz() : VL53L8CX ULD API
//----------------------------------------------------------------

int vl53l8cx_main()
{
    // n is sensor number(0-10)
    // m is frequency(1-60)
    // k is return value of get_Vcp()
    // In   The platform.cpp
  DEBUG_PRINT("main run!!!\n");
  int     n, m, k, InitError;
  sleep_ms(1000);
  sleep_ms(500);
  init_IO();      // In The platform.cpp
  sleep_ms(500);
    printf("TOF Sens Test Start\n");

    InitError = 1;
    // このループは、11個のセンサーがすべて初期化できるまで繰り返す
    while(InitError) {
        for(n = 0, InitError = 0; n < 11; n++) {
            if(Init_Sensor(n, 1) == 0) InitError = 1;
        }
  sleep_ms(500);
    }
    printf("Ranging Start\n");
    for(n = 0; n < 11; n++) {
        vl53l8cx_start_ranging(&MDev[n]);
    }

    while (true) {
        Gget_Ranging();
        k = get_Vcp();
        if(k != 0) {
            //-------------------------------------------------------------------
            // Sampling rate Setup
            // f frequency[1--60]
            //-------------------------------------------------------------------
            if(ucmd[0] == 'f') {
                for( n = 1; ucmd[n] == ' ' || ucmd[n] == '\t'; n++);
                sscanf(&ucmd[n], "%d", &m);
                if(m >= 1 && m <= 60) {
                    for(n = 0; n < 11; n++) {
                        MDev[n].platform.address = n;
                        vl53l8cx_set_ranging_frequency_hz(&MDev[n], m);
                    }
                    printf("Sampling Rate Setup[f=%d]\n", m);
                } else {
                    printf("Error Sampling Rate Setup[f %d]\n", m);
                }
            }
        }
    }
}

// main end
#endif /* host-only example */

#/***********************************************
 * メモリ／RAM 戦略と設定マクロ
 * - VL11_USE_CCM: 大きなバッファを CCM に置いてメイン SRAM を節約します。
 * - VL11_DEFAULT_RATE_HZ: センサあたりのデフォルト測定周波数（多重化前）
 ***********************************************/
#ifndef VL11_USE_CCM
#define VL11_USE_CCM 1
#endif
#ifndef VL11_DEFAULT_RATE_HZ
#define VL11_DEFAULT_RATE_HZ 10
#endif
/* Debug: Limit number of sensors to initialize for testing (0 = all 11) 
 * Set to 1 or 2 for initial hardware testing */
#ifndef VL11_DEBUG_MAX_SENSORS
#define VL11_DEBUG_MAX_SENSORS 11  // Scan all 11 sensor addresses
#endif
#include "vl11_arena.h"

/*
 * ULD バイナリ配列の extern 宣言を取り込むヘッダ
 * - 実体 (VL53L8CX_FIRMWARE 等) は別ファイルで定義されている想定です。
 * - printBlobAddresses() でこれらが FLASH に置かれているか確認します。
 */
#include "vl53l8cx_buffers.h"

/*
 * チップセレクト (CS) ピン配列
 * - 実際のハード配線に合わせてここを設定してください。
 * - g_vl11_cs[i] がセンサ i の CS を表します。
 */
deckPin_t g_vl11_cs[VL11_NUM_SENSORS] = {
  (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0},
  (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0}, (deckPin_t){0},
};

/*
 * ドライバ状態変数
 * - g_running: ドライバ有効化フラグ（パラメータから制御）
 * - g_rate_hz: 測距周波数（Hz）
 * - g_testGen: テスト用擬似ジェネレータを使うか
 * - g_inited: 初期化完了フラグ
 * - g_tick: 内部カウンタ（ログ送信等に利用）
 * - g_blobsOk: ULD バイナリが FLASH 上にあるか
 * - g_initOk: 各センサの初期化結果コード
 */
static volatile uint8_t g_running = 0;
static uint8_t  g_rate_hz = VL11_DEFAULT_RATE_HZ;
static uint8_t  g_testGen = 1;
static uint8_t  g_inited  = 0;
static uint32_t g_tick    = 0;

/* Heap snap + diagnostics */
static uint8_t  g_heapSnap = 0;
static uint8_t  g_blobsOk  = 0;   /* 1 if blobs look valid & in flash */
static uint8_t  g_initOk[VL11_NUM_SENSORS] = {0};

/* Build-time switch to bypass ULD init for isolation tests (0 = skip, 1 = call init) */
#ifndef VL11_CALL_INIT
#define VL11_CALL_INIT 1
#endif

/* Request re-initialization flag (set by forceInit param) */
static volatile uint8_t g_reinitRequested = 0;

/*
 * 共有構造体と結果バッファ
 * - g_dev: VL53L8CX API の設定用構造体（各センサに対して address をセットして使う）
 * - g_res: 測距結果バッファ（大きいため CCM に配置することがある）
 * - g_ranges_mm: 最終的に外部に公開する各センサの距離(mm)
 */
static VL53L8CX_Configuration g_dev;
#ifdef VL11_USE_CCM
__attribute__((section(".ccmram")))
#endif
static VL53L8CX_ResultsData g_res;

/* public last distances */
static uint16_t g_ranges_mm[VL11_NUM_SENSORS];


/*
 * ヒープ監視ユーティリティ
 * - デバッグ用に現在の free / minimum ever を出力します。
 */

/* Runtime CS override params storage and updater -------------------------------------------------- */
/* Per-sensor CS param array holds the deckPin_t.id for each sensor; updated from host params. */
static uint8_t g_cs_param[VL11_NUM_SENSORS] = {0};
/* Update the runtime g_vl11_cs mapping from g_cs_param[]. Called when csN params change. */
static void onCsUpdated(void) {
  for (int _i = 0; _i < VL11_NUM_SENSORS; _i++) {
    g_vl11_cs[_i].id = g_cs_param[_i];
  }
}
/* Force-init helper (set from host to force initialization even if deck not detected) */
static uint8_t g_forceInit = 0;
/* forward-declare the callback; real implementation placed later so it can
   reference task/print helpers that are defined below. */
static void onForceInit(void);

/* ===== Heap probe helper ===== */

static inline void heapSnap(const char* tag) {
  size_t cur = xPortGetFreeHeapSize();
  size_t min = xPortGetMinimumEverFreeHeapSize();
  DEBUG_PRINT("HEAP[%s] cur=%u minEver=%u\n", (tag?tag:""), (unsigned)cur, (unsigned)min);
}

/*
 * CS (Chip Select) のヘルパ
 * - センサインデックスを受け取り対応する CS を操作します。
 */
static inline void cs_low(int i)  { VL11_GPIO_WRITE(g_vl11_cs[i], LOW); }
static inline void cs_high(int i) { VL11_GPIO_WRITE(g_vl11_cs[i], HIGH); }

// /*
//  * ULD (Ultra Low-level Driver) 用 SPI トランスポート実装
//  * - VL53L8CX API が期待する低レイヤの read/write 関数を実装します。
//  * - 各関数は p->address をセンサインデックスとして扱い、該当 CS をトグルします。
//  */
// uint8_t VL53L8CX_WrByte(VL53L8CX_Platform* p, uint16_t reg, uint8_t value) {
//   const int i = (int)p->address;
//   VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
//   uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
//   VL11_SPI_SEND(2, hdr); VL11_SPI_SEND(1, &value);
//   cs_high(i); VL11_SPI_RELEASE(); return 0;
// }
// uint8_t VL53L8CX_RdByte(VL53L8CX_Platform* p, uint16_t reg, uint8_t* p_value) {
//   const int i = (int)p->address;
//   VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
//   uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
//   VL11_SPI_SEND(2, hdr); VL11_SPI_RECV(1, p_value);
//   cs_high(i); VL11_SPI_RELEASE(); return 0;
// }
// uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform* p, uint16_t reg, uint8_t* p_values, uint32_t size) {
//   const int i = (int)p->address;
//   VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
//   uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
//   VL11_SPI_SEND(2, hdr); if (size) VL11_SPI_SEND(size, p_values);
//   cs_high(i); VL11_SPI_RELEASE(); return 0;
// }
// uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform* p, uint16_t reg, uint8_t* p_values, uint32_t size) {
//   const int i = (int)p->address;
//   VL11_SPI_ACQUIRE(); VL11_SPI_START(NULL); cs_low(i);
//   uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
//   VL11_SPI_SEND(2, hdr); if (size) VL11_SPI_RECV(size, p_values);
//   cs_high(i); VL11_SPI_RELEASE(); return 0;
// }
// uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform* p, uint32_t TimeMs) { (void)p; vTaskDelay(pdMS_TO_TICKS(TimeMs)); return 0; }
// void    VL53L8CX_SwapBuffer(uint8_t* buffer, uint16_t size) { (void)buffer; (void)size; }

/* ===== pick a representative distance ===== */
static uint16_t pick_distance_mm(const VL53L8CX_ResultsData* r) {
  /* Use the API resolution macro for the number of zones (4x4 = 16).
     Some ULD drops may disable per-zone counters (nb_target_detected), so
     provide a fallback that scans per-target distances. Also handle the
     case where distance_mm is disabled by returning 0. */
#ifndef VL53L8CX_DISABLE_DISTANCE_MM
  const int zones = (int)VL53L8CX_RESOLUTION_4X4;
  uint16_t best = 0xFFFF;
  for (int z = 0; z < zones; z++) {
#ifndef VL53L8CX_DISABLE_NB_TARGET_DETECTED
    /* Prefer the lightweight per-zone indicator when present */
    if (r->nb_target_detected[z] > 0) {
      uint16_t d = (uint16_t)r->distance_mm[z * VL53L8CX_NB_TARGET_PER_ZONE + 0];
      if (d && d < best) best = d;
    }
#else
    /* Fallback: scan per-target distances for this zone */
    for (int t = 0; t < VL53L8CX_NB_TARGET_PER_ZONE; t++) {
      int idx = z * VL53L8CX_NB_TARGET_PER_ZONE + t;
      int16_t dd = r->distance_mm[idx];
      if (dd > 0 && (uint16_t)dd < best) best = (uint16_t)dd;
    }
#endif
  }
  return (best == 0xFFFF) ? 0 : best;
#else
  (void)r;
  /* distance measurements disabled in this ULD build; nothing to pick */
  return 0;
#endif
}

/* ===== worker ===== */
static void vl11Task(void* arg) {
  (void)arg;
  heapSnap("task-start");

  /* Setup CS pins idle high */
  /* NOTE: CS pin initialization is now handled by platform_c.c initGPIO()
   * which is called from platform.c init_IO(). The 360-degree TOF sensor
   * uses CS0 (IO_4), CS1 (IO_3), and CS2 (IO_2) for device addressing. */
  // for (int i = 0; i < VL11_NUM_SENSORS; i++) {
  //   VL11_GPIO_INIT_OUTPUT(g_vl11_cs[i]);
  //   VL11_GPIO_WRITE(g_vl11_cs[i], HIGH);
  // }
  heapSnap("after-cs-setup");

  /* init sensors sequentially (shared g_dev), one at a time with delays */
  int max_sensors = (VL11_DEBUG_MAX_SENSORS > 0 && VL11_DEBUG_MAX_SENSORS < VL11_NUM_SENSORS) 
                    ? VL11_DEBUG_MAX_SENSORS : VL11_NUM_SENSORS;
  DEBUG_PRINT("vl11Task: Initializing %d sensor(s)...\n", max_sensors);
  
  for (int i = 0; i < max_sensors && g_running; i++) {
    DEBUG_PRINT("vl8cx[%d]: Starting initial init (SKIPPED - waiting for forceInit)...\n", i);
    heapSnap("before-sensor-init");
    
    memset(&g_dev, 0, sizeof(g_dev));
    g_dev.platform.address = (uint16_t)i;
    DEBUG_PRINT("vl8cx[%d]: Set platform.address=%d\n", i, g_dev.platform.address);

#if VL11_CALL_INIT
    if (0) { // SKIP initial init completely - only do re-init after forceInit
      DEBUG_PRINT("vl8cx[%d]: Resetting arena (12KB)...\n", i);
      vl11_arena_reset(12*1024);   // start with 12 KB; adjust if you see OOM log
      
      /* Give system a chance to breathe */
      vTaskDelay(pdMS_TO_TICKS(10));
      
      DEBUG_PRINT("vl8cx[%d]: Calling vl53l8cx_init() with address=%d...\n", i, g_dev.platform.address);
      int8_t st = vl53l8cx_init(&g_dev);
          g_initOk[i] = (st == VL53L8CX_STATUS_OK) ? 1 : (uint8_t)st;  /* store error code too */
          DEBUG_PRINT("vl8cx[%d]: init returned %d\n", i, (int)st);      if (st == VL53L8CX_STATUS_OK) {
        /* Force 4x4 to shrink result buffers */
        DEBUG_PRINT("vl8cx[%d]: Setting resolution to 4x4...\n", i);
        vl53l8cx_set_resolution(&g_dev, VL53L8CX_RESOLUTION_4X4);
        
        /* If your ULD exposes it, keep 1 target/zone (API name differs across drops)
           Examples:
           // vl53l8cx_set_nb_target_per_zone(&g_dev, 1);
           // vl53l8cx_set_nb_targets_per_zone(&g_dev, 1);
        */
        DEBUG_PRINT("vl8cx[%d]: Setting ranging frequency to %d Hz...\n", i, g_rate_hz);
        vl53l8cx_set_ranging_frequency_hz(&g_dev, g_rate_hz);
        
        DEBUG_PRINT("vl8cx[%d]: Starting ranging...\n", i);
        (void)vl53l8cx_start_ranging(&g_dev);
        
        DEBUG_PRINT("vl8cx[%d]: Initial init SUCCESS!\n", i);
      } else {
        DEBUG_PRINT("vl8cx[%d]: Initial init FAILED with status %d\n", i, (int)st);
      }
    } else {
      DEBUG_PRINT("vl8cx[%d]: skipped init (blobs missing)\n", i);
    }
#else
    /* Skip ULD init for isolation */
    vTaskDelay(pdMS_TO_TICKS(2));
#endif

    heapSnap("after-sensor-init");
    
    /* Delay between sensors to avoid overwhelming the system */
    DEBUG_PRINT("vl8cx[%d]: Waiting before next sensor...\n", i);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  g_inited = 1;
  heapSnap("after-all-init");

  for (;;) {
    if (g_heapSnap) { g_heapSnap = 0; heapSnap("param"); }

    /* Check if re-initialization is requested (via forceInit param) */
    if (g_reinitRequested) {
      g_reinitRequested = 0;
      int max_sensors = (VL11_DEBUG_MAX_SENSORS > 0 && VL11_DEBUG_MAX_SENSORS < VL11_NUM_SENSORS) 
                        ? VL11_DEBUG_MAX_SENSORS : VL11_NUM_SENSORS;
      DEBUG_PRINT("vl11Task: re-initializing %d sensor(s) (one at a time)...\n", max_sensors);
      
      /* Re-initialize sensors one by one with delays to avoid watchdog/stack issues */
      for (int i = 0; i < max_sensors && g_running; i++) {
        DEBUG_PRINT("vl8cx[%d]: Starting re-init...\n", i);
        heapSnap("reinit-before");
        
        memset(&g_dev, 0, sizeof(g_dev));
        g_dev.platform.address = (uint16_t)i;
        DEBUG_PRINT("vl8cx[%d]: Set platform.address=%d for reinit\n", i, g_dev.platform.address);

#if VL11_CALL_INIT
        if (g_blobsOk) {
          DEBUG_PRINT("vl8cx[%d]: Resetting arena...\n", i);
          vl11_arena_reset(12*1024);
          
          /* Give watchdog and other tasks a chance to breathe */
          vTaskDelay(pdMS_TO_TICKS(100));  /* Increased delay before init */
          
          /* Test basic sensor communication before full init */
          DEBUG_PRINT("vl8cx[%d]: Testing basic SPI read (device ID)...\n", i);
          uint8_t device_id = 0;
          uint8_t model_id = 0;
          if (VL53L8CX_RdByte(&g_dev.platform, 0x010F, &device_id) == VL53L8CX_STATUS_OK) {
            DEBUG_PRINT("vl8cx[%d]: Device ID read: 0x%02X\n", i, device_id);
          } else {
            DEBUG_PRINT("vl8cx[%d]: Failed to read device ID\n", i);
          }
          if (VL53L8CX_RdByte(&g_dev.platform, 0x010E, &model_id) == VL53L8CX_STATUS_OK) {
            DEBUG_PRINT("vl8cx[%d]: Model ID read: 0x%02X\n", i, model_id);
          } else {
            DEBUG_PRINT("vl8cx[%d]: Failed to read model ID\n", i);
          }
          
          /* SKIP vl53l8cx_init() to avoid crash - focus on hardware detection first */
          DEBUG_PRINT("vl8cx[%d]: SKIPPING vl53l8cx_init() to prevent crash\n", i);
          DEBUG_PRINT("vl8cx[%d]: Hardware scan complete. Device ID: 0x%02X, Model ID: 0x%02X\n", 
                      i, device_id, model_id);
          int8_t st = VL53L8CX_STATUS_ERROR; // Pretend init failed
          g_initOk[i] = 0;
          
          /* Yield to allow system tasks to process */
          vTaskDelay(pdMS_TO_TICKS(10));
          
          if (0) { // Never enter - skip all config
            DEBUG_PRINT("vl8cx[%d]: Setting resolution...\n", i);
            vl53l8cx_set_resolution(&g_dev, VL53L8CX_RESOLUTION_4X4);
            
            DEBUG_PRINT("vl8cx[%d]: Setting ranging frequency...\n", i);
            vl53l8cx_set_ranging_frequency_hz(&g_dev, g_rate_hz);
            
            DEBUG_PRINT("vl8cx[%d]: Starting ranging...\n", i);
            vl53l8cx_start_ranging(&g_dev);
            
            DEBUG_PRINT("vl8cx[%d]: Init SUCCESS!\n", i);
          } else {
            DEBUG_PRINT("vl8cx[%d]: Init FAILED with status %d\n", i, (int)st);
          }
        } else {
          DEBUG_PRINT("vl8cx[%d]: reinit skipped (blobs missing)\n", i);
        }
#endif
        heapSnap("reinit-after");
        
        /* Delay between sensors to avoid overwhelming the system */
        DEBUG_PRINT("vl8cx[%d]: Waiting before next sensor...\n", i);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      DEBUG_PRINT("vl11Task: re-initialization complete for all sensors\n");
    }

    if (g_running && !g_testGen) {
      for (int i = 0; i < VL11_NUM_SENSORS; i++) {
        g_dev.platform.address = (uint16_t)i;
        uint8_t ready = 0;
        if (vl53l8cx_check_data_ready(&g_dev, &ready) == VL53L8CX_STATUS_OK && ready) {
          if (vl53l8cx_get_ranging_data(&g_dev, &g_res) == VL53L8CX_STATUS_OK) {
            g_ranges_mm[i] = pick_distance_mm(&g_res);
          }
        }
      }
    } else if (g_testGen) {
      for (int i = 0; i < VL11_NUM_SENSORS; i++) {
        g_ranges_mm[i] = (uint16_t)(200 + ((g_tick * 7 + i * 93) % 1500));
      }
    }

    g_tick++;
    vTaskDelay(pdMS_TO_TICKS(VL11_POLL_PERIOD_MS));
  }
}

/* Task handle (created on enable toggle) */
static TaskHandle_t g_task = NULL;
/* Optional host entry and optional host init: declare weak so they're safe
 * if not linked. Placed just before the callback that may call them so the
 * prototypes are visible. */
extern int vl53l8cx_main(void) __attribute__((weak));
extern void init_IO(void) __attribute__((weak));

static void onEnableUpdated() {
  // ここは動いている
  heapSnap(g_running ? "enable=1" : "enable=0");
  /* Call host init only if a host stub was linked in (platform.o). This
   * avoids an undefined reference when building the embedded firmware
   * without the host-only platform implementation. */
  if (init_IO) {
    init_IO();
  }

  /* Optionally call the host example entrypoint if it was linked in. The
   * symbol is weak so it's safe to test at runtime. */
  /* Print the pointer value for diagnosis, then call if present */
  DEBUG_PRINT("vl53l8cx11: vl53l8cx_main ptr=%u\n", (unsigned)(uintptr_t)vl53l8cx_main);
  if (vl53l8cx_main) {
    /* If a host/example was linked in, call it. This is optional and may
     * block if that implementation runs a loop — prefer starting the driver
     * task below which is the normal firmware behaviour. */
    vl53l8cx_main();
  }

  /* Normal firmware behaviour: start the vl11 task when enabled so the
   * driver runs in its own FreeRTOS task and publishes ranges. This is the
   * recommended path for embedded operation. */
  DEBUG_PRINT("onEnableUpdated: g_running=%d g_task=%p\n", g_running, (void*)g_task);
  if (g_running && g_task == NULL) {
    DEBUG_PRINT("onEnableUpdated: Creating vl11Task with stack size 1024 words\n");
    /* Increased stack from 768 to 1024 words to handle VL53L8CX init + SPI overhead */
    xTaskCreate(vl11Task, "vl11", 1024, NULL, tskIDLE_PRIORITY + 1, &g_task);
    DEBUG_PRINT("onEnableUpdated: Task created, g_task=%p\n", (void*)g_task);
  }
  // if (g_running && g_task == NULL) {
  // if (g_task == NULL) {
    
  //   xTaskCreate(vl11Task, "vl11", 384, NULL, tskIDLE_PRIORITY + 1, &g_task);
  // }
}

/* onForceInit implementation is provided below after printBlobAddresses() */

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

/* Now the force-init implementation; placed after printBlobAddresses so the
   function is visible (avoids implicit-declaration warnings/errors). */
static void onForceInit(void) {
  heapSnap("forceInit");
  if (g_forceInit) {
    /* Print/check blob addresses and mark blobs OK so vl11Task will attempt init */
    printBlobAddresses();
    g_blobsOk = 1;
    /* Request re-initialization in the task loop */
    g_reinitRequested = 1;
    DEBUG_PRINT("forceInit: set g_blobsOk=1, g_reinitRequested=1\n");
    /* If enabled but task not running, start it */
    // if (g_running && g_task == NULL) {
    if (g_task == NULL) {
      xTaskCreate(vl11Task, "vl11", 384, NULL, tskIDLE_PRIORITY + 1, &g_task);
    }
  }
}

/* ===== Params / Logs ===== */
PARAM_GROUP_START(vl11)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, enable,  &g_running, onEnableUpdated)
  PARAM_ADD(PARAM_UINT8, rate_hz, &g_rate_hz)
  PARAM_ADD(PARAM_UINT8, testGen, &g_testGen)
  PARAM_ADD(PARAM_UINT8, heapsnap, &g_heapSnap)  /* write 1 to print heap */
  PARAM_ADD(PARAM_UINT8, blobs_ok, &g_blobsOk)   /* read-only diagnostic */
  /* Per-sensor CS pin override (deckPin_t.id). Set from host to match your deck wiring.
     Default 0 means unused; update via param e.g. 'vl11.cs0'..'vl11.cs10'. */
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs0, &g_cs_param[0], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs1, &g_cs_param[1], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs2, &g_cs_param[2], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs3, &g_cs_param[3], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs4, &g_cs_param[4], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs5, &g_cs_param[5], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs6, &g_cs_param[6], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs7, &g_cs_param[7], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs8, &g_cs_param[8], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs9, &g_cs_param[9], onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, cs10,&g_cs_param[10],onCsUpdated)
  PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, forceInit, &g_forceInit, onForceInit)
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
  /* Apply any CS overrides set via params before tasks run */
  onCsUpdated();
}

bool vl11IsRunning(void) { return g_running != 0; }
uint16_t vl11GetLastMm(int index) { if (index < 0 || index >= VL11_NUM_SENSORS) return 0; return g_ranges_mm[index]; }

static const DeckDriver bcVL53L8CX11 = { .name = "bcVL53L8CX11", .init = vl11Init };
DECK_DRIVER(bcVL53L8CX11);
