//----------------------------------------------------------------
// C言語版 TOF Sensor Test
//----------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l8cx_api.h"
#include "vl53l8cx_buffers.h"
#include "platform.h"
#include "debug.h"

/* 
 * HAL層の関数プロトタイプ - ユーザー実装が必要
 */

/* シリアル通信関数 */
extern void HAL_Serial_Init(void);
extern int HAL_Serial_Readable(void);
extern int HAL_Serial_Read(char *buf, int size);
extern int HAL_Serial_Write(const char *str, int len);

/* タイマー/遅延関数 */
extern void HAL_Delay_ms(uint32_t ms);
extern uint32_t HAL_GetTick_ms(void);

/* printf用のラッパー */
int serial_printf(const char *format, ...);

VL53L8CX_Configuration Dev;
VL53L8CX_ResultsData Results;  /* Results data from VL53L8CX */

//---------------------------------------------------------------------------
//  Serial Input
//---------------------------------------------------------------------------
char ucmd[64];
int ucmd_p = 0;

int get_Vcp(void)
{
    int n, k;
    char buf[64];

    k = HAL_Serial_Readable();
    if(k == 0) return 0;
    k = HAL_Serial_Read(buf, sizeof(buf));
    if(k != 0) {
        for(n = 0; n < k; n++) {
            if(buf[n] == '\r' || buf[n] == '\n') {
                ucmd[ucmd_p] = 0;
                ucmd_p = 0;
                return k;
            } else {
                ucmd[ucmd_p] = buf[n];
                if(ucmd_p < 63) ucmd_p++;
            }
        }
    }
    return 0;
}

//----------------------------------------------------------------
// Example_1_Ranging_Basic(The VL53L8CX ULD package)
//----------------------------------------------------------------
void Ranging_Basic(uint16_t DevAddr)
{
    uint8_t status, loop, isAlive, isReady, i;
    VL53L8CX_Configuration Dev_local;

    Dev_local.platform.address = DevAddr;

    /* (Optional) Check if there is a VL53L8CX sensor connected */
    status = vl53l8cx_is_alive(&Dev_local, &isAlive);
    if(!isAlive || status) {
        DEBUG_PRINT("VL53L8CX not detected at requested address\n");
        return;
    }

    /* (Mandatory) Init VL53L8CX sensor */
    status = vl53l8cx_init(&Dev_local);
    if(status) {
        DEBUG_PRINT("VL53L8CX ULD Loading failed\n");
        return;
    }
    DEBUG_PRINT("VL53L8CX ULD ready ! (Version : %s)\n", VL53L8CX_API_REVISION);

    /* Ranging loop */
    status = vl53l8cx_set_ranging_frequency_hz(&Dev_local, 1);
    if(status) {
        DEBUG_PRINT("vl53l8cx_set_ranging_frequency_hz failed, status %u\n", status);
        return;
    }
    status = vl53l8cx_start_ranging(&Dev_local);
    loop = 0;
    while(loop < 10) {
        status = vl53l8cx_check_data_ready(&Dev_local, &isReady);
        if(isReady) {
            vl53l8cx_get_ranging_data(&Dev_local, &Results);
            DEBUG_PRINT("Print data no : %3u\n", Dev_local.streamcount);
            for(i = 0; i < 16; i++) {
                DEBUG_PRINT("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
                    Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE*i],
                    Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
            }
            DEBUG_PRINT("\n");
            loop++;
        }
        VL53L8CX_WaitMs(&(Dev_local.platform), 5);
    }
}

//----------------------------------------------------------------
// Multiple Sensor(by Kizaki)
//----------------------------------------------------------------
VL53L8CX_Configuration MDev[11];

int Init_Sensor(uint16_t DevAddr, uint8_t Frequency)
{
    uint8_t status, isAlive;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_is_alive(&MDev[DevAddr], &isAlive);
    if(status) {
        DEBUG_PRINT("VL53L8CX ULD Loading failed_alive[%d]\n", DevAddr);
        return 0;
    }
    status = vl53l8cx_init(&MDev[DevAddr]);
    if(status) {
        DEBUG_PRINT("VL53L8CX ULD Loading failed_init[%d]\n", DevAddr);
        return 0;
    }
    DEBUG_PRINT("VL53L8CX ULD ready ! (Version : %s)[%d]\n", VL53L8CX_API_REVISION, DevAddr);

    status = vl53l8cx_set_ranging_frequency_hz(&MDev[DevAddr], Frequency);
    if(status) {
        DEBUG_PRINT("vl53l8cx_set_ranging_frequency_hz failed, status %u[%d]\n", status, DevAddr);
        return 0;
    }
    return 1;
}

void Start_Ranging(uint16_t DevAddr)
{
    uint8_t status;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_start_ranging(&MDev[DevAddr]);
    (void)status; /* 未使用変数の警告抑制 */
}

uint8_t DevAddr[11];
uint8_t ReStart[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t NumRdy[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void Gget_Ranging(void)
{
    uint8_t status, loop, isAlive, isReady;
    int i;
    int k;

    for(k = 0; k < 11; k++) DevAddr[k] = 0xFF;
    k = Ser_IT();       /* In The platform.c */
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
            DEBUG_PRINT("[%2d] ", DevAddr[k]);
            for(i = 0; i < 16; i++) {
                if(Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i] == 5) {
                    DEBUG_PRINT("[%4d]", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*i]);
                } else {
                    DEBUG_PRINT("--%02X--", Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i]);
                }
            }
            DEBUG_PRINT(" R[%3d]", NumRdy[k]);
            DEBUG_PRINT(" S[%d]", ReStart[k]);
            DEBUG_PRINT("\n");
            NumRdy[k] = 0;
        } else {
            NumRdy[k]++;
        }
    }
    
    for(k = 0; k < 11; k++) {
        if(NumRdy[k] > 250) {
            DEBUG_PRINT("Restart DevAddr[%d] NumRdy[%d]\n", k, NumRdy[k]);
            Start_Ranging(k);
            ReStart[k]++;
        }
    }
    
    (void)status;
    (void)loop;
    (void)isAlive;
    (void)isReady;
}

//----------------------------------------------------------------
// Main
//----------------------------------------------------------------
int vl53l8cx_main(void)
{
    int n, m, k, InitError;
    
    HAL_Delay_ms(500);
    init_IO();      /* In The platform.c */
    HAL_Delay_ms(500);
    DEBUG_PRINT("TOF Sens Test Start\n");

    InitError = 1;
    while(InitError) {
        for(n = 0, InitError = 0; n < 11; n++) {
            if(Init_Sensor(n, 1) == 0) InitError = 1;
        }
        HAL_Delay_ms(500);
    }
    DEBUG_PRINT("Ranging Start\n");
    for(n = 0; n < 11; n++) {
        vl53l8cx_start_ranging(&MDev[n]);
    }

    while (1) {
        Gget_Ranging();
        k = get_Vcp();
        if(k != 0) {
            /* Sampling rate Setup */
            /* f frequency[1--60] */
            if(ucmd[0] == 'f') {
                for(n = 1; ucmd[n] == ' ' || ucmd[n] == '\t'; n++);
                sscanf(&ucmd[n], "%d", &m);
                if(m >= 1 && m <= 60) {
                    for(n = 0; n < 11; n++) {
                        MDev[n].platform.address = n;
                        vl53l8cx_set_ranging_frequency_hz(&MDev[n], m);
                    }
                    DEBUG_PRINT("Sampling Rate Setup[f=%d]\n", m);
                } else {
                    DEBUG_PRINT("Error Sampling Rate Setup[f %d]\n", m);
                }
            }
        }
    }
    
    return 0;
}
