/**
 *  @file    mysdcard.c
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

// std
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>

// esp-idf general
#include <esp_log.h>
#include <esp_err.h>

// drivers
#include <driver/sdmmc_host.h>
#include <driver/sdspi_host.h>

// components
#include <sdmmc_cmd.h>
#include <esp_vfs_fat.h>

// private
#include <mysdcard.h>

/*
 * Constants/Statics/Globals
 */
// Debug tag
static const char *TAG = "MYSDCARD";
sdmmc_card_t *card;

 
/*
 * Start the esp32 uSD card driver and connect
 * it to the AlarmDecoder ESP32 Display board pins
 */
bool mysdcard_init() {
  ESP_LOGI(TAG, "uSD start mounting HSPI on pins");
  
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  slot_config.gpio_miso = SPISD_PIN_NUM_MISO;
  slot_config.gpio_mosi = SPISD_PIN_NUM_MOSI;
  slot_config.gpio_sck  = SPISD_PIN_NUM_CLK;
  slot_config.gpio_cs   = SPISD_PIN_NUM_CS;
  
  // Options for mounting the filesystem.
  // If format_if_mount_failed is set to true, SD card will be partitioned and
  // formatted in case when mounting fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
     .format_if_mount_failed = false,
     .max_files = 5,
     .allocation_unit_size = 16 * 1024
  };

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
  // Please check its source code and implement error recovery when developing
  // production applications.
  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
     if (ret == ESP_FAIL) {
         ESP_LOGE(TAG, "Failed to mount filesystem. "
             "If you want the card to be formatted, set format_if_mount_failed = true.");
     } else {
         ESP_LOGE(TAG, "Failed to initialize the card (%s). "
             "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
     }
     return false;
  }

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, card);
  
  return true;
}

void mysdcard_uninit() {
  // All done, unmount partition and disable SDMMC or SPI peripheral
  esp_vfs_fat_sdmmc_unmount();
  ESP_LOGI(TAG, "uSD unmounted");  
}
