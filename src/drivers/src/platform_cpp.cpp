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

#include <cstdint>
#include <stdlib.h>
#include <string.h>
#include "platform.h"
/* Include the VL53L8CX platform interface so C types like VL53L8CX_Platform
 * and the VL53L8CX_* function prototypes are visible to this C++ file. We
 * use the relative path into the drivers interface directory. */
#include "../interface/platform.h"

// Provide two alternatives: the original mbed implementation when USE_MBED
// is defined, otherwise fallback to lightweight GCC-friendly stubs that
// allow compilation on a normal host toolchain. The stubs are simple and
// intended as placeholders; replace thinit_IOem with real hardware bindings as
// needed (e.g., spidev, sysfs GPIO, wiringPi, libgpiod, etc.).

#ifndef USE_MBED
#include <thread>
#include <chrono>

// Define simple integer constants for pin names used in the original code.
#define PA_7  0
#define PA_6  1
#define PB_3  2
#define PB_12 3
#define PB_6  4
#define PC_7  5

// Minimal SPI stub: echo writes back the last written byte by default.
class SPI {
public:
    SPI(int mosi, int miso, int sck) {}
    void format(int bits, int mode) { (void)bits; (void)mode; }
    void frequency(int hz) { (void)hz; }
    // write a single byte and return a byte as response
    uint8_t write(uint8_t data) { last = data; return last; }
private:
    uint8_t last = 0;
};

// Minimal DigitalOut stub
class DigitalOut {
public:
    DigitalOut(int pin) : value(0), pin(pin) { (void)pin; }
    void operator=(int v) { value = (v != 0); }
    operator int() const { return value ? 1 : 0; }
private:
    bool value;
    int pin;
};

// Minimal DigitalIn stub (returns 1 by default: not asserted)
class DigitalIn {
public:
    DigitalIn(int pin) : pin(pin) { (void)pin; }
    operator int() const { return 1; }
private:
    int pin;
};

// Provide ThisThread::sleep_for(TimeMs) compatibility
namespace ThisThread {
    inline void sleep_for(uint32_t ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}

// Instantiate the stubs with the same names used by the codebase.
SPI         Spi(PA_7, PA_6, PB_3);
DigitalOut  CS0(PB_12);
DigitalOut  CS1(PB_6);
DigitalIn   CS2(PC_7);      // Interrupted: Measurement completed

#else // USE_MBED

// If building with mbed, the original declarations are expected from mbed.h
#include "mbed.h"

#endif // USE_MBED

// This function is called once at startup
// It initializes the SPI and the GPIOs
void init_IO()
{
    Spi.format(8, 3);
    Spi.frequency(2500000);
    CS0 = 1;
    CS1 = 1;
}

volatile uint16_t BckDev = 0xFFFF;

uint16_t Ser_IT()
{
    static uint16_t Intr;

    if(CS2 == 0) return 0;
    CS0 = 0;
    Intr  = (uint16_t)Spi.write((uint8_t)(BckDev >> 8)) << 8;
    Intr |= (uint16_t)Spi.write((uint8_t)(BckDev & 0xFF));
    CS0 = 1;
    return Intr;
}

void Sel_Dev(unsigned short Dev)
{
    uint8_t    rD;

    if(Dev != BckDev) {
        CS0 = 0;
        rD = Spi.write((uint8_t)((Dev >> 8) & 0xFF));
        rD = Spi.write((uint8_t)(Dev & 0xFF));
        CS0 = 1;
        BckDev = Dev;
    }
}

uint8_t VL53L8CX_RdByte(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_value)
{
    unsigned char    rD[3];
    uint8_t status = 255;
    /* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD[0] = Spi.write((uint8_t)(RegisterAdress >> 8));
    rD[1] = Spi.write((uint8_t)(RegisterAdress & 0x00FF));
    rD[2] = Spi.write(0x00);
    CS1 = 1;
    *p_value = rD[2];
    status = 0;
    return status;
}

uint8_t VL53L8CX_WrByte(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t value)
{
    unsigned char    rD[3];
    uint8_t         status = 255;
    /* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD[0] = Spi.write((uint8_t)((RegisterAdress >> 8) | 0x80));
    rD[1] = Spi.write((uint8_t)(RegisterAdress & 0x00FF));
    rD[2] = Spi.write(value);
    CS1 = 1;
    status = 0;
    return status;
}

uint8_t VL53L8CX_WrMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    int             n;
    unsigned char   rD;
    uint8_t status = 255;

    /* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD = Spi.write((uint8_t)((RegisterAdress >> 8) | 0x80));
    rD = Spi.write((uint8_t)(RegisterAdress & 0x00FF));
    for(n = 0; n < (int)size; n++) {
        rD = Spi.write(p_values[n]);
    }
    CS1 = 1;
    status = 0;
    return status;
}

// this function is called by API functions
// status is updated with I2C error status
uint8_t VL53L8CX_RdMulti(
        VL53L8CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    int             n;
    unsigned char   rD;
    uint8_t status = 255;

    /* Need to be implemented by customer. This function returns 0 if OK */
    Sel_Dev(p_platform->address);
    CS1 = 0;
    rD = Spi.write((uint8_t)(RegisterAdress >> 8));
    rD = Spi.write((uint8_t)(RegisterAdress & 0x00FF));
    for(n = 0; n < (int)size; n++) {
        rD = Spi.write(0x00);
        *(p_values + n) = rD;
    }
    CS1 = 1;
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
        uint8_t         *buffer,
        uint16_t         size)
{
    uint32_t i, tmp;

    /* Example of possible implementation using <string.h> */
    for(i = 0; i + 3 < size; i = i + 4)
    {
        tmp = (
          ((uint32_t)buffer[i] << 24)
        | ((uint32_t)buffer[i+1] << 16)
        | ((uint32_t)buffer[i+2] << 8)
        | ((uint32_t)buffer[i+3]));

        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t VL53L8CX_WaitMs(
        VL53L8CX_Platform *p_platform,
        uint32_t TimeMs)
{
    uint8_t status = 255;

    /* Need to be implemented by customer. This function returns 0 if OK */
    ThisThread::sleep_for(TimeMs);
    status = 0;
    return status;
}
