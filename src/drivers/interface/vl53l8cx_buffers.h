
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



#define VL53L8CX_BUFFERS_H_

#include "platform.h"
#pragma once
#include <stdint.h>
/**
 * @brief Inner internal number of targets.
 */

#if VL53L8CX_NB_TARGET_PER_ZONE == 1
#define VL53L8CX_FW_NBTAR_RANGING	2
#else
#define VL53L8CX_FW_NBTAR_RANGING	VL53L8CX_NB_TARGET_PER_ZONE
#endif

/**
 * @brief This buffer contains the VL53L8CX firmware (MM1.8)
 */
 
extern const uint8_t VL53L8CX_FIRMWARE[];
extern const uint8_t VL53L8CX_DEFAULT_CONFIGURATION[];
extern const uint8_t VL53L8CX_DEFAULT_XTALK[];
extern const uint8_t VL53L8CX_GET_NVM_CMD[];
