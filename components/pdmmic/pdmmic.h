/**
 *  @file    pdmmic.h
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    09/28/2018
 *  @version 1.0
 *
 *  @brief API to communicate with PDM based mic from a ESP32 uP
 *
 *   This code provides an API for audio recording via a MEMS PDM
 *  based microphone (SPH0641LU4H-1) connected to an ESP32. It 
 *  will collect the PDM data and decimate it in real time to a
 *  PCM stream. The clock speed frequency can be adjusted to put
 *  the microphone in different power and frequency modes.
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

#ifndef _PDMMIC_H_
#define _PDMMIC_H_

/**
 * Defines
 */

// General settings/utils

// Default i2s sample rate
#define PDMMIC_LP_SAMPLE_RATE  8000   // 516.1kHz LOW POWER 270uA
#define PDMMIC_MP_SAMPLE_RATE 11000   // 711.1kHz LOW POWER 270uA
#define PDMMIC_SP_SAMPLE_RATE 22050   // 1.455MHz STANDARD 700uA
#define PDMMIC_HD_SAMPLE_RATE 44100   // 2.667MHz HIGH DEFINITION 1000uA

// Wiring
#define PDMMIC_CLK_PIN   CONFIG_PDMMIC_CLK_PIN   //_GPIO_NUM_26
#define PDMMIC_DATA_PIN  CONFIG_PDMMIC_DATA_PIN  //_GPIO_NUM_35

// RTOS PDM consumer task
#define PDMMIC_CLIENT_TASK_NAME       "pdmmic_client_task"
#define PDMMIC_CLIENT_TASK_STACK      5120
#define PDMMIC_CLIENT_PRIORITY        7

/**
 * Public Prototypes
 */

// Initialize the ESP32 i2s device driver and connect them to the pins for the MEMS device
bool pdmmic_init();

// Start processing PDM data
bool pdmmic_start(uint32_t rate);

// Stop processing PDM data
bool pdmmic_stop();

// stop free pdmmic driver resources
void pdmmic_deinit();


#endif
