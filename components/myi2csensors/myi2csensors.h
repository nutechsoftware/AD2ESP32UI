/**
 *  @file    myi2csensors.h
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    02/18/2019
 *  @version 1.0
 *
 *  @brief API to allow for communication to multiple i2c sensors.
 *
 *   Provides an API for managing a shared i2s sensor bus
 *  on the AD2ESP32 board. This manages the i2c bus init and cleanup
 *  as well as hooks for talking to sensors on the bus.
 *
 *  @copyright Copyright (C) 2018-2019 Nu Tech Software Solutions, Inc.
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

#ifndef _MYI2CSENSORS_H_
#define _MYI2CSENSORS_H_

/**
 * Defines
 */

// General settings/utils


// Wiring
#define MYI2CSENSORS_CLK_PIN   CONFIG_MYI2CSENSORS_CLK_PIN   //_GPIO_NUM_2
#define MYI2CSENSORS_DATA_PIN  CONFIG_MYI2CSENSORS_DATA_PIN  //_GPIO_NUM_4

// RTOS PDM consumer task
#define MYI2CSENSORS_CLIENT_TASK_NAME       "myi2csensors_client_task"
#define MYI2CSENSORS_CLIENT_TASK_STACK      5120
#define MYI2CSENSORS_CLIENT_PRIORITY        7

/**
 * Public Prototypes
 */

// Initialize the ESP32 i2c device driver and connect them to the pins for the MEMS device
bool myi2csensors_init();

// Start processing myi2csensors data
bool myi2csensors_start(uint32_t speed);

// Stop processing myi2csensors data
bool myi2csensors_stop();

// stop free myi2csensors driver resources
void myi2csensors_deinit();


#endif
