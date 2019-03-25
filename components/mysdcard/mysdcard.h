/**
 *  @file    mysdcard.h
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    10/01/2018
 *  @version 1.0
 *
 *  @brief Private uSD card functions for the AD2ESP32UI
 *
 *  @copyright Copyright (C) 2018 Nu Tech Software Solutions, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef _MYSDCARD_H_
#define _MYSDCARD_H_
#include "freertos/task.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"

/**
 * Defines
 */

// Wiring
#define SPISD_PIN_NUM_MISO  CONFIG_SPISD_PIN_NUM_MISO  // GPIO_NUM_12 // GREEN   D0 | D0 
#define SPISD_PIN_NUM_MOSI  CONFIG_SPISD_PIN_NUM_MOSI  // GPIO_NUM_13 // YELLOW CMD | DI
#define SPISD_PIN_NUM_CLK   CONFIG_SPISD_PIN_NUM_CLK   // GPIO_NUM_14 // PURPLE CLK
#define SPISD_PIN_NUM_CS    CONFIG_SPISD_PIN_NUM_CS    // GPIO_NUM_15 // WHITE   D3 | CS
#define SPISD_PIN_NUM_CD    CONFIG_SPISD_PIN_NUM_CD    // GPIO_NUM_39 //
#define SPISD_PIN_NUM_EN    CONFIG_SPISD_PIN_NUM_EN    // GPIO_NUM_27 //

/**
 * Public Prototypes
 */

// init sdcard / vfat driver hardware
bool mysdcard_init();

// start the sdcard driver
bool mysdcard_start();

// stop the sdcard driver
bool mysdcard_stop();

// stop and free sdcard driver resources
bool mysdcard_deinit();

// set the power state to the uSD card
bool mysdcard_setpower(uint8_t powerstate);

#endif
