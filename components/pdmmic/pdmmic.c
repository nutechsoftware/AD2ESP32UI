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
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_system.h"
 #include "esp_spi_flash.h"
 #include "driver/spi_master.h"
 #include "soc/gpio_struct.h"
 #include "driver/gpio.h"
 #include "driver/i2s.h"
 #include "esp_log.h"
 #include "pdmmic.h"
 
 /*
  * Constants/Statics/Globals
  */
 // Debug tag
 static const char *TAG = "PDMMIC";
 
 
// internal only prototypes
static void pdmmic_client_task(void *p);
// fast count of bits
inline uint16_t count_bits(uint32_t v);



 bool pdmmic_init() {

   ESP_LOGI(TAG, "PDM MIC. INIT.");
   
   i2s_config_t i2s_config = {
           .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM,    //master, RX, PDM
           .sample_rate = SAMPLE_RATE,
           .bits_per_sample = BITS_PS,
           .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
           .communication_format = I2S_COMM_FORMAT_PCM, //pcm data format
           .dma_buf_count = BUF_COUNT,
           .dma_buf_len = BUF_LENGTH,
           .use_apll = 0,                               //apll disabled
           .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1     //interrupt level 1(lowest priority)
   };
   
   i2s_pin_config_t pin_config = {
           .ws_io_num = PDMMIC_CLK_PIN,
           .data_in_num = PDMMIC_DATA_PIN,
   };

   i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
   i2s_set_pin(I2S_NUM, &pin_config);
   
   vTaskDelay(1000/portTICK_PERIOD_MS);
   uint32_t cnt = 0;
   uint16_t buffer = 0;
   size_t read = 0;
   while(1)
   {
      cnt++;
      buffer = 0;
      
      int bytes_popped = i2s_read(I2S_NUM, &buffer, sizeof(buffer), &read, 10);
      uint16_t cb = count_bits(buffer);
      if(bytes_popped) {
        printf("%d", cb);
      }
   }
}


bool pdmmic_start() {
  esp_err_t ret;
  xTaskHandle pdmmic_client_handle;
  
  ret = xTaskCreate(pdmmic_client_task,
                    PDMMIC_CLIENT_TASK_NAME,
                    PDMMIC_CLIENT_TASK_NAME_STACK,
                    NULL,
                    8,
                    &pdmmic_client_handle);

  if (ret != pdPASS)  {
      ESP_LOGI(TAG, "create thread %s failed", PDMMIC_CLIENT_TASK_NAME);
      return false;
  }
  return true;
}

// http://graphics.stanford.edu/~seander/bithacks.html
inline uint16_t count_bits(uint32_t v) {
  v = v - ((v >> 1) & 0x55555555);                    // reuse input as temporary
  v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
  uint32_t c = (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24; // count
  return (uint16_t)c;
}

#if 0 // TODO R&D alg name refernce. 
/* Function to get no of set bits in binary 
   representation of passed binary no. */
uint16_t count_bits(uint16_t n) 
{ 
    uint16_t count = 0; 
    while (n) 
    { 
      n &= (n-1) ; 
      count++; 
    } 
    return count; 
}
#endif

/*
 * PDM data processing task reads PDM data 
 * and preforms the first decimation by counting the bits.
 */
static void pdmmic_client_task(void *p)
{
  while(1) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
  // never happens
  return;
}
