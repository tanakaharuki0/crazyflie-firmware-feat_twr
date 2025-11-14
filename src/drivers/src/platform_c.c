/**
 * platform_c.c - VL53L8CX HAL implementation for Crazyflie using deck SPI API
 *
 * This implementation uses the Crazyflie deck SPI and GPIO APIs to communicate
 * with VL53L8CX sensors. The 360-degree TOF sensor deck uses:
 * - CS0 (IO_4): Device address selection
 * - CS1 (IO_3): Actual sensor communication chip select
 * - CS2 (IO_2): Interrupt input (measurement complete)
 *
 * Based on "360Deg TOFsensor Operating Instructions.pdf" page 2.
 */

#include <stdint.h>
#include <stdbool.h>
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32fxxx.h"
#include "stm32f4xx_spi.h"
#include "debug.h"

/* CS pin definitions based on hardware wiring */
static deckPin_t CS0_PIN;  // IO_4 - Device address selection
static deckPin_t CS1_PIN;  // IO_3 - Sensor chip select
static deckPin_t CS2_PIN;  // IO_2 - Interrupt (measurement complete)

/* Shared with platform.c for device selection */
extern volatile uint16_t BckDev;

/* Current SPI configuration */
static uint16_t g_spi_prescaler = SPI_BaudRatePrescaler_64;  // ~1.3MHz default
static bool g_spi_initialized = false;
static bool g_gpio_initialized = false;

/* Debug: Enable detailed logging of sensor selection (disable after debugging) */
#define DEBUG_SENSOR_SELECTION 0

void HAL_SPI_Init(void) {
    if (!g_spi_initialized) {
        spiBegin();
        g_spi_initialized = true;
    }
}

static void initGPIO(void) {
    if (!g_gpio_initialized) {
        /* Initialize CS pin constants based on PDF page 2
         * NOTE: PDF specifies actual STM32 pins (PA5, PA2, PC15)
         * which differ from standard DECK_GPIO_IOx mappings.
         * Using standard deck mappings for now:
         *   DECK_GPIO_IO4 (id=7) = PC12
         *   DECK_GPIO_IO3 (id=6) = PB4
         *   DECK_GPIO_IO2 (id=5) = PB5
         * TODO: Verify actual hardware wiring! */
        CS0_PIN = DECK_GPIO_IO4;  // PC12 - Device address selection
        CS1_PIN = DECK_GPIO_IO3;  // PB4 - Sensor chip select
        CS2_PIN = DECK_GPIO_IO2;  // PB5 - Interrupt input
        
        /* Configure CS0 and CS1 as outputs, initially high */
        pinMode(CS0_PIN, OUTPUT);
        pinMode(CS1_PIN, OUTPUT);
        digitalWrite(CS0_PIN, HIGH);
        digitalWrite(CS1_PIN, HIGH);
        
        /* Configure CS2 as input for interrupt */
        pinMode(CS2_PIN, INPUT_PULLUP);
        
        g_gpio_initialized = true;
    }
}

void HAL_SPI_SetFormat(int mode, int bits) {
    /* SPI format is fixed by deck_spi.c to Mode 0, 8 bits */
    (void)mode;
    (void)bits;
}

void HAL_SPI_SetFrequency(unsigned int hz) {
    /* Map requested frequency to closest prescaler */
    /* Based on 84MHz peripheral clock:
       SPI_BaudRatePrescaler_8  = 10.5 MHz
       SPI_BaudRatePrescaler_16 = 5.25 MHz
       SPI_BaudRatePrescaler_32 = 2.625 MHz
       SPI_BaudRatePrescaler_64 = 1.3125 MHz */
    if (hz >= 10000000) {
        g_spi_prescaler = SPI_BaudRatePrescaler_8;
    } else if (hz >= 5000000) {
        g_spi_prescaler = SPI_BaudRatePrescaler_16;
    } else if (hz >= 2500000) {
        g_spi_prescaler = SPI_BaudRatePrescaler_32;
    } else {
        g_spi_prescaler = SPI_BaudRatePrescaler_64;
    }
}

void HAL_GPIO_Init(void) {
    /* Initialize GPIO pins for CS0, CS1, CS2 */
    initGPIO();
}

void HAL_GPIO_WriteCS0(uint8_t value) {
    /* CS0 (IO_4) - Device address selection via SPI
     * Used by platform.c Sel_Dev() to send device address to the multiplexer
     * CS0 LOW = begin address selection, CS0 HIGH = latch address */
    if (!g_gpio_initialized) {
        initGPIO();
    }
    
    if (value == 0) {
        /* CS0 active (low) - begin address selection transaction */
        digitalWrite(CS0_PIN, LOW);
        if (g_spi_initialized) {
            spiBeginTransaction(g_spi_prescaler);
        }
    } else {
        /* CS0 inactive (high) - latch the address */
        if (g_spi_initialized) {
            spiEndTransaction();
        }
        digitalWrite(CS0_PIN, HIGH);
    }
}

void HAL_GPIO_WriteCS1(uint8_t value) {
    /* CS1 (IO_3) - Actual sensor chip select
     * Controls SPI communication with the currently selected sensor 
     * CS1 LOW = begin SPI transaction, CS1 HIGH = end SPI transaction */
    static uint32_t cs1_toggle_count = 0;
    
    if (!g_gpio_initialized) {
        initGPIO();
    }
    
    if (value == 0) {
        /* CS active (low) - begin SPI transaction */
        digitalWrite(CS1_PIN, LOW);
        if (g_spi_initialized) {
            spiBeginTransaction(g_spi_prescaler);
        }
        cs1_toggle_count++;
        if (cs1_toggle_count <= 5) {
            DEBUG_PRINT("[CS1] LOW (begin transaction #%lu)\n", cs1_toggle_count);
        }
    } else {
        /* CS inactive (high) - end SPI transaction */
        if (g_spi_initialized) {
            spiEndTransaction();
        }
        digitalWrite(CS1_PIN, HIGH);
        if (cs1_toggle_count <= 5) {
            DEBUG_PRINT("[CS1] HIGH (end transaction #%lu)\n", cs1_toggle_count);
        }
    }
}

uint8_t HAL_GPIO_ReadCS2(void) {
    /* CS2 (IO_2) - Interrupt input (measurement complete)
     * Returns 1 when sensor measurement is complete, 0 otherwise */
    if (!g_gpio_initialized) {
        initGPIO();
    }
    return (uint8_t)digitalRead(CS2_PIN);
}

uint8_t HAL_SPI_Write(uint8_t data) {
    uint8_t rx_byte = 0;
    static uint32_t spi_call_count = 0;
    
    if (!g_spi_initialized) {
        HAL_SPI_Init();
    }
    
    /* Note: Do NOT call spiBeginTransaction/spiEndTransaction here!
     * The transaction is managed by the CS pin (HAL_GPIO_WriteCS1).
     * CS1 LOW = begin transaction, CS1 HIGH = end transaction.
     * Multiple HAL_SPI_Write() calls happen within a single CS transaction. */
    spiExchange(1, &data, &rx_byte);
    
    /* Debug: Log first few SPI transactions */
    spi_call_count++;
    if (spi_call_count <= 10) {
        DEBUG_PRINT("[SPI] #%lu TX:0x%02X RX:0x%02X\n", spi_call_count, data, rx_byte);
    }
    
    return rx_byte;
}

void HAL_Delay_ms(uint32_t ms) {
    /* Use FreeRTOS delay */
    vTaskDelay(pdMS_TO_TICKS(ms));
}
