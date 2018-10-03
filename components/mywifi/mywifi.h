/**
 *  @file    myswifi.h
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    10/01/2018
 *  @version 1.0
 *
 *  @brief Private wifi/network functions for the AD2ESP32UI
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

#ifndef _MYWIFI_H_
#define _MYWIFI_H_


/**
 * Defines
 */

/* WIFI settings */
#define AD2ESP32UI_WIFI_SSID          CONFIG_ESP_WIFI_SSID
#define AD2ESP32UI_WIFI_PASS          CONFIG_ESP_WIFI_PASSWORD
#define AD2ESP32UI_MAXIMUM_RETRY      10
#define MAX_HTTP_RECV_BUFFER          512

/* WEBCAM MJPEG collector task */
#define MJPEG_CLIENT_TASK_NAME        "mjpeg_client_task"
#define MJPEG_CLIENT_TASK_NAME_STACK  10240


/**
 * Public Prototypes
 */
 
// PUBLIC WIFI FUNCTIONS
bool mywifi_init();
bool mywifi_start();
bool mywifi_stop();
bool mywifi_deinit();

// PUBLIC CAM STREAM FUNCTIONS
bool mjpegcam_init();
bool mjpegcam_start();
bool mjpegcam_stop();
bool mjpegcam_deinit();

#endif
