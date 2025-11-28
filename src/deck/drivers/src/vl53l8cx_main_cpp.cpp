//----------------------------------------------------------------
// mbed6 NUCLEO-F446RE
//----------------------------------------------------------------
// #include    "mbed.h"
// #include    "mbedthread.h"
#include <cstdint>
#include    <stdlib.h>
#include    <string.h>
#include    "vl53l8cx_api.h"
#include    "vl53l8cx_buffers.h"
#include    "platform.h"
// #include <stdio.h>  // ARM embeddedでは使用しない
#include "vl53l8cx_main_cpp.h"

#ifndef USE_MBED
// ARM embedded (Crazyflie) - FreeRTOSとCrazyflieのAPIを使用
#include "FreeRTOS.h"
#include "task.h"
#include "../../../utils/interface/debug.h"

// FreeRTOS版のsleep (std::this_thread::sleep_forの代替)
namespace ThisThread {
    inline void sleep_for(uint32_t ms) {
        vTaskDelay(M2T(ms));  // FreeRTOSのdelay
    }
}

// BufferedSerial の代替 (Crazyflieではシリアル入力不要)
class BufferedSerial {
public:
    BufferedSerial(int tx, int rx, int baud) { 
        (void)tx; (void)rx; (void)baud; 
    }
    // ARM embedded環境では常に0を返す (入力なし)
    int readable() {
        return 0;  // シリアル入力なし
    }
    // ARM embedded環境では何も読まない
    int read(char *buf, size_t size) {
        (void)buf; (void)size;
        return 0;  // 読み込みなし
    }
};

// instantiate stub (Crazyflie embedded用)
static BufferedSerial serial_vcp(0, 0, 115200);

#else
static BufferedSerial serial_vcp(PA_2, PA_3, 115200);
#endif
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

    k = serial_vcp.readable();
    if(k == 0) return(0);
    k = serial_vcp.read(buf, sizeof(buf));
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
		// DEBUG_PRINT("VL53L8CX not detected at requested address\n");
		return;
	}

    // (Mandatory) Init VL53L8CX sensor
	status = vl53l8cx_init(&Dev);
	if(status) {
		// DEBUG_PRINT("VL53L8CX ULD Loading failed\n");
		return;
	}
    // DEBUG_PRINT("VL53L8CX ULD ready ! (Version : %s)\n", VL53L8CX_API_REVISION);

    // Ranging loop
    status = vl53l8cx_set_ranging_frequency_hz(&Dev, 1);
	if(status) {
		// DEBUG_PRINT("vl53l8cx_set_ranging_frequency_hz failed, status %u\n", status);
		return;
	}
    status = vl53l8cx_start_ranging(&Dev);
    loop = 0;
	while(loop < 10) {
        status = vl53l8cx_check_data_ready(&Dev, &isReady);
        if(isReady) {
			vl53l8cx_get_ranging_data(&Dev, &Results);
            // DEBUG_PRINT("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++) {
				// DEBUG_PRINT("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
					// Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
					// Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
			}
			// DEBUG_PRINT("\n");
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
		// DEBUG_PRINT("VL53L8CX ULD Loading failed_alive[%d]\n", DevAddr);
		return(0);
	}
    status = vl53l8cx_init(&MDev[DevAddr]);
	if(status) {
		// DEBUG_PRINT("VL53L8CX ULD Loading failed_init[%d]\n", DevAddr);
		return(0);
	}
    // DEBUG_PRINT("VL53L8CX ULD ready ! (Version : %s)[%d]\n", VL53L8CX_API_REVISION, DevAddr);

    status = vl53l8cx_set_ranging_frequency_hz(&MDev[DevAddr], Frequency);
	if(status) {
		// DEBUG_PRINT("vl53l8cx_set_ranging_frequency_hz failed, status %u[%d]\n", status, DevAddr);
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
            //// DEBUG_PRINT("[%d]Print data no : %3u\n", DevAddr[k], MDev[DevAddr[k]].streamcount);
            // DEBUG_PRINT("[%2d] ", DevAddr[k]);
			for(i = 0; i < 16; i++) {
                if(Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i] == 5) {
                    // DEBUG_PRINT("[%4d]", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
                } else {
                    // DEBUG_PRINT("--%02X--", Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i]);
                }
			}
            // DEBUG_PRINT(" R[%3d]", NumRdy[k]);
            // DEBUG_PRINT(" S[%d]", ReStart[k]);
			// DEBUG_PRINT("\n");
            NumRdy[k] = 0;
        } else {
            NumRdy[k]++;
        }
    }
    for(k = 0; k < 11; k++) {
        if(NumRdy[k] > 250) {
            // DEBUG_PRINT("Restart DevAddr[%d] NumRdy[%d]\n", k, NumRdy[k]);
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

extern "C" int vl53l8cx_main()
{
    // n is sensor number(0-10)
    // m is frequency(1-60)
    // k is return value of get_Vcp()
    // In   The platform.cpp
    // DEBUG_PRINT("main run!!!\n");
    int     n, m, k, InitError;
    ThisThread::sleep_for(1000);
    ThisThread::sleep_for(500);
    init_IO();      // In The platform.cpp
    ThisThread::sleep_for(500);
    // DEBUG_PRINT("TOF Sens Test Start\n");

    InitError = 1;
    // このループは、11個のセンサーがすべて初期化できるまで繰り返す
    while(InitError) {
        for(n = 0, InitError = 0; n < 11; n++) {
            if(Init_Sensor(n, 1) == 0) InitError = 1;
        }
    ThisThread::sleep_for(500);
    }
    // DEBUG_PRINT("Ranging Start\n");
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
                // sscanf(&ucmd[n], "%d", &m);  // ARM embedded: scanf unavailable
                m = 10;  // デフォルト値
                if(m >= 1 && m <= 60) {
                    for(n = 0; n < 11; n++) {
                        MDev[n].platform.address = n;
                        vl53l8cx_set_ranging_frequency_hz(&MDev[n], m);
                    }
                    // DEBUG_PRINT("Sampling Rate Setup[f=%d]\n", m);
                } else {
                    // DEBUG_PRINT("Error Sampling Rate Setup[f %d]\n", m);
                }
            }
        }
    }
}
