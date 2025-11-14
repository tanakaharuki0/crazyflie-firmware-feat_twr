/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../interface/platform.h"
#include "debug.h"

/* 
 * HAL層の関数プロトタイプ
 * ユーザーは実際のハードウェアに合わせてこれらの関数を実装する必要があります
 */

/* SPI関数 - ユーザー実装が必要 */
extern void HAL_SPI_Init(void);
extern void HAL_SPI_SetFormat(uint8_t bits, uint8_t mode);
extern void HAL_SPI_SetFrequency(uint32_t frequency);
extern uint8_t HAL_SPI_Write(uint8_t data);

/* GPIO関数 - ユーザー実装が必要 */
extern void HAL_GPIO_Init(void);
extern void HAL_GPIO_WriteCS0(uint8_t value);
extern void HAL_GPIO_WriteCS1(uint8_t value);
extern uint8_t HAL_GPIO_ReadCS2(void);

/* タイマー関数 - ユーザー実装が必要 */
extern void HAL_Delay_ms(uint32_t ms);

volatile uint16_t BckDev = 0xFFFF;

void init_IO(void)
{
    HAL_SPI_Init();
    HAL_SPI_SetFormat(8, 3);
    HAL_SPI_SetFrequency(2500000);
    HAL_GPIO_Init();
    HAL_GPIO_WriteCS0(1);
    HAL_GPIO_WriteCS1(1);
}

uint16_t Ser_IT(void)
{
    static uint16_t Intr;

    if(HAL_GPIO_ReadCS2() == 0) return 0;
    
    HAL_GPIO_WriteCS0(0);
    Intr  = HAL_SPI_Write((uint8_t)(BckDev >> 8)) << 8;
    Intr |= HAL_SPI_Write(0x00); 
    HAL_GPIO_WriteCS0(1);
    return Intr;
}

void Sel_Dev(unsigned short Dev)
{
    uint8_t rD;
    static uint32_t sel_dev_count = 0;

    if(Dev != BckDev) {
        sel_dev_count++;
        if (sel_dev_count <= 3) {
            DEBUG_PRINT("[Sel_Dev] Switching to device %u (count=%lu)\n", Dev, sel_dev_count);
        }
        
        /* Select sensor by sending device address via CS0/SPI */
        HAL_GPIO_WriteCS0(0);
        rD = HAL_SPI_Write((uint8_t)(Dev >> 8));
        rD = HAL_SPI_Write((uint8_t)(Dev & 0xFF));
        HAL_GPIO_WriteCS0(1);
        BckDev = Dev;
        
        /* Small delay to allow multiplexer to settle */
        HAL_Delay_ms(1);
        
        if (sel_dev_count <= 3) {
            DEBUG_PRINT("[Sel_Dev] Device %u selected, waiting 1ms\n", Dev);
        }
    }
    (void)rD; /* 未使用変数の警告抑制 */
}

uint8_t VL53L8CX_RdByte(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_value)
{
    uint8_t status = 255;
    
    /* VL53L8CX SPI Read Protocol:
     * 1. Send 16-bit address (MSB must be 0 for read)
     * 2. Send dummy byte and read response
     * Note: Some implementations require CS toggle between address and data */
    
    Sel_Dev(p_platform->address);
    
    /* Transaction 1: Send address */
    HAL_GPIO_WriteCS1(0);
    HAL_SPI_Write((uint8_t)((RegisterAdress >> 8) & 0x7F));  // MSB=0 for read
    HAL_SPI_Write((uint8_t)(RegisterAdress & 0xFF));
    HAL_GPIO_WriteCS1(1);
    
    /* Small delay to allow sensor to prepare data */
    HAL_Delay_ms(1);
    
    /* Transaction 2: Read data */
    HAL_GPIO_WriteCS1(0);
    *p_value = HAL_SPI_Write(0x00);  // Send dummy byte, read response
    HAL_GPIO_WriteCS1(1);
    
    status = 0;
    return status;
}

uint8_t VL53L8CX_WrByte(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t value)
{
    uint8_t rD[3];    
    uint8_t status = 255;
    
    /* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    HAL_GPIO_WriteCS1(0);
    rD[0] = HAL_SPI_Write((uint8_t)((RegisterAdress >> 8) | 0x80));
    rD[1] = HAL_SPI_Write((uint8_t)(RegisterAdress & 0x00FF));
    rD[2] = HAL_SPI_Write(value);
    HAL_GPIO_WriteCS1(1);
    status = 0;
    (void)rD; /* 未使用変数の警告抑制 */
    return status;
}

uint8_t VL53L8CX_WrMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    uint32_t n;
    uint8_t rD;
    uint8_t status = 255;
    
    /* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    HAL_GPIO_WriteCS1(0);
    rD = HAL_SPI_Write((uint8_t)((RegisterAdress >> 8) | 0x80));
    rD = HAL_SPI_Write((uint8_t)(RegisterAdress & 0x00FF));
    for(n = 0; n < size; n++) {
        rD = HAL_SPI_Write(p_values[n]);
    }
    HAL_GPIO_WriteCS1(1);
    status = 0;
    (void)rD; /* 未使用変数の警告抑制 */
    return status;
}

uint8_t VL53L8CX_RdMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    uint32_t n;
    uint8_t rD;
    uint8_t status = 255;
    
    /* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    HAL_GPIO_WriteCS1(0);
    rD = HAL_SPI_Write((uint8_t)(RegisterAdress >> 8));
    rD = HAL_SPI_Write((uint8_t)(RegisterAdress & 0x00FF));
    for(n = 0; n < size; n++) {
        rD = HAL_SPI_Write(0x00);
        *(p_values + n) = rD;
    }
    HAL_GPIO_WriteCS1(1);
    status = 0;
    return status;
}

uint8_t VL53L8CX_Reset_Sensor(
        VL53L8CX_Platform *p_platform)
{
    uint8_t status = 0;
    
    /* (Optional) Need to be implemented by customer. This function returns 0 if OK */
    
    /* Set pin LPN to LOW */
    /* Set pin AVDD to LOW */
    /* Set pin VDDIO  to LOW */
    /* Set pin CORE_1V8 to LOW */
    VL53L8CX_WaitMs(p_platform, 100);

    /* Set pin LPN to HIGH */
    /* Set pin AVDD to HIGH */
    /* Set pin VDDIO to HIGH */
    /* Set pin CORE_1V8 to HIGH */
    VL53L8CX_WaitMs(p_platform, 100);

    return status;
}

void VL53L8CX_SwapBuffer(
        uint8_t *buffer,
        uint16_t size)
{
    uint32_t i, tmp;
    
    /* Example of possible implementation using <string.h> */
    for(i = 0; i < size; i = i + 4) 
    {
        tmp = (
          (uint32_t)buffer[i]<<24)
        |((uint32_t)buffer[i+1]<<16)
        |((uint32_t)buffer[i+2]<<8)
        |((uint32_t)buffer[i+3]);
        
        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t VL53L8CX_WaitMs(
        VL53L8CX_Platform *p_platform,
        uint32_t TimeMs)
{
    uint8_t status = 255;

    /* Need to be implemented by customer. This function returns 0 if OK */
    HAL_Delay_ms(TimeMs);
    status = 0;
    (void)p_platform; /* 未使用パラメータの警告抑制 */
    return status;
}
