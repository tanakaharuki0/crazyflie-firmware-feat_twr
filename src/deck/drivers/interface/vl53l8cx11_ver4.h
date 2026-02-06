
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "deck_digital.h"
#include "deck_spi.h"
#include "param.h"

#ifndef VL11_NUM_SENSORS
#define VL11_NUM_SENSORS 1
#endif
#ifndef VL11_DEFAULT_RATE_HZ
#define VL11_DEFAULT_RATE_HZ 10
#endif
#ifndef VL11_POLL_PERIOD_MS
#define VL11_POLL_PERIOD_MS 20
#endif

#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif

/* SPI helpers (override if you use custom names) */
#ifndef VL11_SPI_ACQUIRE
#define VL11_SPI_ACQUIRE() ((void)0)
#endif
#ifndef VL11_SPI_RELEASE
#define VL11_SPI_RELEASE() ((void)0)
#endif
#ifndef VL11_SPI_START
#define VL11_SPI_START(cfg)     \
    do                          \
    {                           \
        (void)(cfg);            \
        spiBegin();             \
        spiBeginTransaction(8); \
    } while (0)
#endif
#ifndef VL11_SPI_SEND
#define VL11_SPI_SEND(n, buf)                                 \
    do                                                        \
    {                                                         \
        (void)spiExchange((n), (const uint8_t *)(buf), NULL); \
    } while (0)
#endif
#ifndef VL11_SPI_RECV
#define VL11_SPI_RECV(n, buf)                           \
    do                                                  \
    {                                                   \
        (void)spiExchange((n), NULL, (uint8_t *)(buf)); \
    } while (0)
#endif
#ifndef VL11_SPI_SEND_WRITE
#define VL11_SPI_SEND_WRITE(n, buf)                           \
    do                                                        \
    {                                                         \
        (void)spiExchange((n), (const uint8_t *)(buf), NULL); \
    } while (0)
#endif
#ifndef VL11_SPI_RECV_WRITE
#define VL11_SPI_RECV_WRITE(n, buf)                     \
    do                                                  \
    {                                                   \
        (void)spiExchange((n), NULL, (uint8_t *)(buf)); \
    } while (0)
#endif
#ifndef VL11_SPI_SEND_READ
#define VL11_SPI_SEND_READ(n, buf)                            \
    do                                                        \
    {                                                         \
        (void)spiExchange((n), (const uint8_t *)(buf), NULL); \
    } while (0)
#endif
#ifndef VL11_SPI_RECV_READ
#define VL11_SPI_RECV_READ(n, buf)                      \
    do                                                  \
    {                                                   \
        (void)spiExchange((n), NULL, (uint8_t *)(buf)); \
    } while (0)
#endif

#ifndef VL11_GPIO_INIT_OUTPUT
#define VL11_GPIO_INIT_OUTPUT(pin) pinMode((pin), OUTPUT)
#endif
#ifndef VL11_GPIO_WRITE
#define VL11_GPIO_WRITE(pin, lvl) digitalWrite((pin), (lvl))
#endif

/* CS lines provided in the .c (fill with real deckPin_t pins) */
extern const deckPin_t g_vl11_cs[VL11_NUM_SENSORS];

/* Public helper API (optional) */
#ifdef __cplusplus
extern "C"
{
#endif
    bool vl11IsRunning(void);
    uint16_t vl11GetLastMm(int index);
#ifdef __cplusplus
}
#endif
