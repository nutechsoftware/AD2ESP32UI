/**
 *  @file    pdmmic.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <driver/spi_master.h>
#include <soc/gpio_struct.h>
#include <driver/gpio.h>
#include <driver/i2s.h>
#include <esp_log.h>
#include <pdmmic.h>
#include <soc/i2s_reg.h>

/**
 * Private Defines
 */ 
//#define PDMMIC_WRITE_TEST_FILE 1

// i2s driver settings for MEMS PDM signal
#define BITS_PS 16
#define I2S_NUM 0
#define BUF_COUNT 32
#define BUF_LENGTH 64

//// Non linear or LOG volume
//// https://www.microchip.com/forums/m932509.aspx
unsigned VolumeLUT[63]=    
{    
    0x0001,0x0001,0x0001,0x0002,0x0002,0x0002,0x0003,0x0004,
    0x0004,0x0005,0x0006,0x0008,0x0009,0x000B,0x000D,0x0010,
    0x0013,0x0016,0x001A,0x0020,0x0026,0x002D,0x0035,0x0040,
    0x004C,0x005A,0x006B,0x0080,0x0098,0x00B5,0x00D7,0x0100,
    0x0130,0x016A,0x01AE,0x0200,0x0260,0x02D4,0x035D,0x0400,
    0x04C1,0x05A8,0x06BA,0x0800,0x0983,0x0B50,0x0D74,0x1000,
    0x1306,0x16A0,0x1AE8,0x2000,0x260D,0x2D41,0x35D1,0x4000,
    0x4C1B,0x5A82,0x6BA2,0x8000,0x9837,0xB504,0xD744
};
 
/**
 * Constants/Statics/Globals
 */

 // GAIN Control 0-62 from VolumeLUT
 uint8_t PDMMIC_GAIN = 20;

// Debug tag
static const char *TAG = "PDMMIC";
 
// pdmmic task handle
xTaskHandle pdmmic_client_handle = NULL;

// private prototypes
static void pdmmic_client_task(void *p);

/***************************
 * PMDMIC FUNCTIONS
 */
 
/**
 * Initialize the pdmmic driver
 */
bool pdmmic_init() 
{
   ESP_LOGI(TAG, "INIT");
   
   return true;
}

/**
 * Start the pdm processing thread
 */
bool pdmmic_start(uint32_t rate) 
{
    esp_err_t ret;
  
    ESP_LOGI(TAG, "START"); 

    // Configure the ESP32 for PDM to PCM conversion from a SPH0641LU4H.
    // The SPH0641LU4H-1 SELECT pin is tied to GND. 
    // so samples are valid on the falling edge of the clock.
    // It is not clear from the ESP32 docs what is considered Left or Right
    // but I2S_CHANNEL_FMT_ONLY_RIGHT seems to work.
    // ~12.4.7 https://www.mouser.com/pdfdocs/ESP32-Tech_Reference.pdf    
    i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM,    //master, RX, PDM->PCM conv.
            .sample_rate = rate,
            .bits_per_sample = BITS_PS,
            .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
            .communication_format = I2S_COMM_FORMAT_I2S,
            .dma_buf_count = BUF_COUNT,
            .dma_buf_len = BUF_LENGTH,
            .use_apll = 0,                               // apll disabled
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1     //interrupt level 1(lowest priority)
    };

    i2s_pin_config_t pin_config = {
            .ws_io_num = PDMMIC_CLK_PIN,
            .data_in_num = PDMMIC_DATA_PIN,
    };
    
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    ret = xTaskCreate(pdmmic_client_task,
                      PDMMIC_CLIENT_TASK_NAME,
                      PDMMIC_CLIENT_TASK_STACK,
                      NULL,
                      PDMMIC_CLIENT_PRIORITY,
                      &pdmmic_client_handle);

    if (ret != pdPASS) {
        ESP_LOGI(TAG, "create thread %s failed", PDMMIC_CLIENT_TASK_NAME);
        return false;
    }
    return true;
}

/**
 * Stop the pdm processing thread
 */
bool pdmmic_stop()
{
    ESP_LOGI(TAG, "STOP");
    
    // if a processing task is running stop it.
    if (pdmmic_client_handle!= NULL) {
        vTaskDelete(pdmmic_client_handle);
        pdmmic_client_handle = NULL;
    }

    // disable the driver stop the clock to the i2s device
    i2s_driver_uninstall(I2S_NUM); //stop & destroy i2s driver

    return true;
}



/**
 * Close the driver
 */
void pdmmic_deinit() 
{
    ESP_LOGI(TAG, "DEINIT");
    return true;
}

/**
 * PDM data processing task reads PCM data do any additional 
 * filtering and gain adjustments.
 */
static void pdmmic_client_task(void *p)
{
    uint32_t sum = 0;
    uint16_t buffer = 0;
    size_t read = 0;

#if PDMMIC_WRITE_TEST_FILE
    FILE* fout;
    fout = fopen("/sdcard/test.pcm", "w+");
    if (fout == NULL) {
       ESP_LOGE(TAG, "Failed to open file for writing");
       return;
    }
    ESP_LOGE(TAG, "test.pcm open for write");
    uint32_t tcount = 0;
#endif

    // sleep for 350ms to let the first N samples drop
    // I get a POP at the start probably due to the sensor 
    // starting up or maybe the ESP32 SINC filtering.
    vTaskDelay(350 / portTICK_PERIOD_MS);

    // Continuously consume 16bits of PCM data at a time
    while(1) {

      read = 0;      
      i2s_read(I2S_NUM, &buffer, sizeof(buffer), &read, portMAX_DELAY);

      if (read) {

        // Adjust gain using LUT with a LOG scale
        sum = buffer;// * VolumeLUT[PDMMIC_GAIN];

#if PDMMIC_WRITE_TEST_FILE
        // write the 16bit PCM sample to the file
        if (fout) {
          fwrite(&sum, 1, 2, fout);
          tcount++;

          if (fout && tcount>80000) {
            #if 1
            fclose(fout);              
            #endif
            fout = 0;
            printf("\nFILE CLOSE\n");
          }
        }
#endif

      }
    }

    // FIXME: never happens
    return;
}
