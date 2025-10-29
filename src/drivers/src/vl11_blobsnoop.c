
#include <stdint.h>
#include "deck.h"
#include "debug.h"

/* Extern-only declarations: make sure your vl53l8cx_buffers.h declares them as 'extern const' */
#include "vl53l8cx_buffers.h"

static void blobAddrInit(DeckInfo* info) {
  (void)info;
  DEBUG_PRINT("VL8CX blob addrs: FW=%p CFG=%p XTALK=%p NVM=%p\n",
              VL53L8CX_FIRMWARE,
              VL53L8CX_DEFAULT_CONFIGURATION,
              VL53L8CX_DEFAULT_XTALK,
              VL53L8CX_GET_NVM_CMD);
  DEBUG_PRINT("Note: FLASH ~0x080xxxxx, SRAM ~0x200xxxxx. Arrays must be in FLASH.\n");
}

static const DeckDriver bcVL8CXBlobSnoop = {
  .name = "bcVL8CXBlobSnoop",
  .init = blobAddrInit,
};
DECK_DRIVER(bcVL8CXBlobSnoop);
