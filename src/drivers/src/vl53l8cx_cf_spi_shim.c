/**
 * vl53l8cx_cf_spi_shim.c  (malloc-free, single-session chunked)
 *
 * BEGIN/END and CS low/high are done ONCE per transfer.
 * Payload is streamed in small chunks to keep DMA buffers tiny,
 * but we do NOT re-enter the SPI layer per chunk (prevents heap churn).
 */

#include <stdint.h>
#include <stddef.h>

/* Forward decl to avoid include-order issues. */
typedef struct VL53L8CX_Configuration VL53L8CX_Configuration;

/* ULD platform prototypes */
#include "platform.h"

/* Backend SPI primitives (from your bridge/backend) */
void CF_VL53_SPI_BEGIN(void);
void CF_VL53_SPI_END(void);
void CF_VL53_CS_LOW(void);
void CF_VL53_CS_HIGH(void);
void CF_VL53_SPI_TXRX(const uint8_t* tx, uint8_t* rx, size_t len);

/* Tunable: keep chunk tiny to limit backend DMA buffers */
#ifndef VLX_SPI_CHUNK
#define VLX_SPI_CHUNK  32
#endif

/* Static small buffers in main SRAM (DMA-capable) */
static uint8_t rx_sink_hdr[2];
static uint8_t rx_sink_chunk[VLX_SPI_CHUNK + 2];
static uint8_t txbuf[VLX_SPI_CHUNK + 2];

/* ---- ULD hooks ----------------------------------------------------------- */
int32_t vl53l8cx_platform_write_multi(VL53L8CX_Configuration* p_dev,
                                      uint16_t reg_index,
                                      uint8_t* pdata,
                                      uint32_t count)
{
  (void)p_dev;
  CF_VL53_SPI_BEGIN();
  CF_VL53_CS_LOW();

  uint32_t remaining = count;
  uint32_t off = 0;
  uint16_t reg = reg_index;

  while (remaining > 0) {
    const uint32_t n = (remaining > VLX_SPI_CHUNK) ? VLX_SPI_CHUNK : remaining;
    txbuf[0] = (uint8_t)(reg >> 8);
    txbuf[1] = (uint8_t)(reg & 0xFF);
    for (uint32_t i = 0; i < n; i++) txbuf[2 + i] = pdata[off + i];
    CF_VL53_SPI_TXRX(txbuf, rx_sink_chunk, n + 2);
    off += n;
    remaining -= n;
    reg = (uint16_t)(reg + n);
  }

  CF_VL53_CS_HIGH();
  CF_VL53_SPI_END();
  return 0;
}

int32_t vl53l8cx_platform_read_multi(VL53L8CX_Configuration* p_dev,
                                     uint16_t reg_index,
                                     uint8_t* pdata,
                                     uint32_t count)
{
  (void)p_dev;
  CF_VL53_SPI_BEGIN();
  CF_VL53_CS_LOW();

  uint32_t remaining = count;
  uint32_t off = 0;
  uint16_t reg = reg_index;

  while (remaining > 0) {
    const uint32_t n = (remaining > VLX_SPI_CHUNK) ? VLX_SPI_CHUNK : remaining;
    /* issue address */
    uint8_t hdr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    CF_VL53_SPI_TXRX(hdr, rx_sink_hdr, 2);
    /* read n bytes; reuse pdata as TX to save RAM */
    CF_VL53_SPI_TXRX(&pdata[off], &pdata[off], n);
    off += n;
    remaining -= n;
    reg = (uint16_t)(reg + n);
  }

  CF_VL53_CS_HIGH();
  CF_VL53_SPI_END();
  return 0;
}

/* Optional single-byte helpers (some ULD releases reference these) */
int32_t vl53l8cx_platform_write_byte(VL53L8CX_Configuration* p_dev,
                                     uint16_t reg_index, uint8_t data) {
  return vl53l8cx_platform_write_multi(p_dev, reg_index, &data, 1);
}

int32_t vl53l8cx_platform_read_byte(VL53L8CX_Configuration* p_dev,
                                    uint16_t reg_index, uint8_t* data) {
  return vl53l8cx_platform_read_multi(p_dev, reg_index, data, 1);
}

