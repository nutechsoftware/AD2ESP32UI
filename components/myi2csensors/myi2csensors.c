/**
 *  @file    my2csensors.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <myi2csensors.h>
#include <byteswap.h>

/**
 * Private Defines
 */ 
#define ACK_VAL       0x0
#define NACK_VAL      0x1
#define ACK_CHECK_EN  0x1  // Master will check for ack bit.
#define ACK_CHECK_DIS 0x0  // Master will not check for ack bit.

/**
 * Constants/Statics/Globals
 */

// myi2csensors task handle
xTaskHandle myi2csensors_client_handle = NULL;

// private prototypes
static void myi2csensors_client_task(void *p);

// Debug tag
static const char *TAG = "MYI2CSENS";
 
// private prototypes

/***************************
 * MYI2CSENS FUNCTIONS
 */
 
/**
 * Initialize the i2c driver
 */
bool myi2csensors_init() 
{
    ESP_LOGI(TAG, "INIT");
     
    return true;
}

/**
 * Start the myi2csensors processing thread
 */
bool myi2csensors_start(uint32_t speed) 
{
    esp_err_t ret;  
    ESP_LOGI(TAG, "START"); 

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = MYI2CSENSORS_DATA_PIN;
    conf.scl_io_num = MYI2CSENSORS_CLK_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = speed;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    int res;
    i2c_get_timeout(I2C_NUM_0, &res);  // Defaults is 6400 derived from I2C_MASTER_TOUT_CNUM_DEFAULT(8)
    i2c_set_timeout(I2C_NUM_0, 32000); // allow for possible 'clock stretching' from the sensor
    
    ESP_LOGI(TAG, "i2c timeout %i", res);

    ret = xTaskCreate(myi2csensors_client_task,
                      MYI2CSENSORS_CLIENT_TASK_NAME,
                      MYI2CSENSORS_CLIENT_TASK_STACK,
                      NULL,
                      MYI2CSENSORS_CLIENT_PRIORITY,
                      &myi2csensors_client_handle);

    if (ret != pdPASS) {
        ESP_LOGI(TAG, "create thread %s failed", MYI2CSENSORS_CLIENT_TASK_NAME);
        return false;
    }
    return true;
}

/**
 * Stop the i2c processing thread
 */
bool myi2csensors_stop()
{
    ESP_LOGI(TAG, "STOP");

    // if a processing task is running stop it.
    if (myi2csensors_client_handle!=NULL) {
        vTaskDelete(myi2csensors_client_handle);
        myi2csensors_client_handle = NULL;
    }

    // disable the driver i2c driver
    i2c_driver_delete(I2C_NUM_0); //stop & destroy i2c driver
    
    return true;
}



/**
 * Close the driver
 */
void myi2csensors_deinit() 
{
    ESP_LOGI(TAG, "DEINIT");
    return true;
}

/**
 * i2c data processing task.
 * Function specifics TBD.
 *    Notes:
 *       Detected new devices and load driver based upon device address?
 */
static void myi2csensors_client_task(void *p)
{
    int i;
    esp_err_t espRc;
    
#if 1
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (i=3; i< 0x7e; i++) {
    	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    	ESP_ERROR_CHECK(i2c_master_start(cmd));
    	i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
    	ESP_ERROR_CHECK(i2c_master_stop(cmd));

    	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    	if (i%16 == 0) {
    		printf("\n%.2x:", i);
    	}
    	if (espRc == 0) {
    		printf(" %.2x", i);
    	} else {
    		printf(" --");
    	}
    	//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
    	i2c_cmd_link_delete(cmd);
    }
    printf("\n");
#endif

#if 1
    vTaskDelay(50 / portTICK_PERIOD_MS);
    #define CCS811_I2C_ADDRESS 0x5b
    #define CCS811_CHIP_ID_REG 0x20
    uint8_t cs811buf[32];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CCS811_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CCS811_CHIP_ID_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( CCS811_I2C_ADDRESS << 1 ) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, cs811buf, NACK_VAL);
            
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
      printf("ACK");
    } else {
      printf("NAK");
    }
    printf("CCS811 CHIP HW_ID: 0x%x\n",cs811buf[0]);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(50 / portTICK_PERIOD_MS);
#endif

    // FIXME: sleep forever just testing.
    while (1) {
        #if 1    
        #define RFID_SLAVE_ADDRESS 0x7d
        uint8_t rfidbuf[12];

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        espRc = i2c_master_start(cmd);
        espRc |= i2c_master_write_byte(cmd, (RFID_SLAVE_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
        espRc |= i2c_master_read(cmd, rfidbuf, 9, ACK_VAL /*ACK Value*/);
        espRc |= i2c_master_read_byte(cmd, rfidbuf+9, NACK_VAL);
        espRc |= i2c_master_stop(cmd);
        
        if (espRc == ESP_OK) {
          espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
          if (espRc == ESP_OK) {
            printf("RFID:");
            for(int a = 0; a < 10; a++) {
              printf("%.2x",rfidbuf[a]);
            }
            printf("\n");
            char outbuffer[20];
            uint32_t timestamp = __bswap_32( *((uint32_t*)(&rfidbuf[6])) );
            uint32_t tagvalue = __bswap_32( *((uint32_t*)(&rfidbuf[1])) );
            snprintf(outbuffer, sizeof(outbuffer), "%u,%u", timestamp, tagvalue);
            ESP_LOGI(TAG, "myi2c-rfid %s", outbuffer);
          } else {
            ESP_LOGI(TAG, "myi2c-rfid read error or T/O");
          }
         	i2c_cmd_link_delete(cmd);

        } else {
          ESP_LOGI(TAG, "myi2c-rfid queue transmit error");
          i2c_cmd_link_delete(cmd);
        }
        #endif

        vTaskDelay(500 / portTICK_PERIOD_MS); 
    }
}
