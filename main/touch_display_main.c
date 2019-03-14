/**
 *  @file    touch_display_main.c
 *  @author  Sean Mathews <coder@f34r.com>
 *  @date    09/30/2018
 *  @version 1.0
 *
 *  @brief AlarmDecoder Touch Screen UI
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
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/dirent.h>
#include <string.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>

// esp-idf general
#include <esp_log.h>
#include <esp_err.h>

// drivers

// components
/// esp-idf specific
#include <nvs_flash.h>
#include <esp_spi_flash.h>
#include <esp_wifi.h>
#include <esp_http_client.h>

/// private
#include <ft81x.h>
#include <pdmmic.h>
#include <mysdcard.h>
#include <mywifi.h>
#include <myi2csensors.h>

/*
 * Defines
 */ 

/*
 * Constants/Statics
 */ 
// Debug tag
static const char* TAG = "AD2ESP32UI";

// VSPI IPC mutex
SemaphoreHandle_t s_vspi_mutex;

// FT81x state vars
extern struct ft81x_ctouch_t ft81x_ctouch;
extern struct ft81x_touch_t ft81x_touch;
extern uint32_t mf_wp;

// CAM stream state
extern uint8_t mjpeg_spool_ready;


/*
 * Types
 */
 
/**
 * Prototypes
 */
  
// Enable GPIO pin for bit bang debugging good for timing tests :c)
bool enable_gpio_debug_pin();

// restart the esp32
void restart_esp32();

// tests spooling video from SD card to FT813
void test_video();


/***************************
 * AD2ESP32UI FUNCTIONS
 */

/**
 * FreeRTOS main() entry point
 */
void app_main()
{
    bool res;

    // Disable buffering on stdout
    setvbuf(stdout, NULL, _IONBF, 0);
    
    printf("%s STARTING app_main()\n", TAG);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // Enable GPIO debug output pin
    enable_gpio_debug_pin();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init the i2c sensor bus
    myi2csensors_init();
    #if 0 // TEST MYI2CSENSORS
    myi2csensors_start(50000);
    //myi2csensors_stop();
    #endif
    
    // Initialize the SPI driver on ESP32 VSPI pins for the FT81X
    res = ft81x_initSPI();
    if (!res) {
      printf("ESP32 SPI init failed\n");
      restart_esp32();
    } else {
      printf("ESP32 SPI init success\n");
    }

#if 1 // Disable GPU for testing
    // Initialize the FT81X GPU
    res = ft81x_initGPU();
    if (!res) {
      printf("FT81X init failed\n");
      restart_esp32();
    } else {
      printf("FT81X init success ID 0x%04x\n", ft81x_chip_id);
    }
#endif


    // Init uSD card
    mysdcard_init();

    // Init MEMS microphone
    pdmmic_init();

    #if 0 // TEST PDMMIC
      mysdcard_start();
      // ft81x_wr(REG_PWM_DUTY, 0); /* RF Noise testing. Turn off PWM */
      pdmmic_start(PDMMIC_SP_SAMPLE_RATE);
      vTaskDelay(24000 / portTICK_PERIOD_MS);
      pdmmic_stop();
      mysdcard_stop();
    #endif

    // Init WiFi hardware
    mywifi_init();
    
    // Connect to WiFi
    mywifi_start();

    // main app screen
    test_video();

    // disconnect from WiFi
    mywifi_stop();

    // close i2c sensors bus
    myi2csensors_deinit();
    
    // close uSD card
    mysdcard_deinit();
    
    // close MEMS microphone
    pdmmic_deinit();
        
    // close WiFi
    mywifi_deinit();
    
    // All done
    restart_esp32();
}

/**
 * Restart ESP32 chip
 */
void restart_esp32() 
{
    for (int i = 4; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}


/**
 * Enable an output pin for bit banging debug data
 */
bool enable_gpio_debug_pin() 
{
    esp_err_t ret;
    gpio_config_t io_conf = {
    .intr_type = GPIO_PIN_INTR_DISABLE,    
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = 1LL << GPIO_NUM_16,
    };

    ret = gpio_config(&io_conf);
    if(ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to add DEBUG pin");
      return false;
    }
    return true;
}

void test_video() 
{

#if 0 // LOAD VIDEO FROM uSD TEST  
    // Start our SD Card
    sdmmc_card_t card;  
    start_sdcard(&card);
    
    ESP_LOGI(TAG, "Opening Video File");
    //FILE* fin = fopen("/sdcard/cbw3a.avi", "r");
    //FILE* fin = fopen("/sdcard/70s_tv09.avi", "r");
    FILE* fin = fopen("/sdcard/Sam720N.avi", "r");  
    if (fin == NULL) {
       ESP_LOGE(TAG, "Failed to open file for reading");
       return;
    }
    
    // Display brightness
    ft81x_wr(REG_PWM_DUTY, 42);

    // Set volume to MAX
    ft81x_wr(REG_VOL_SOUND,0x00);
    ft81x_wr(REG_VOL_PB,0xff);
    
    // Turn on the audio amp connected to GPIO3
    ft81x_wr16(REG_GPIOX_DIR, ft81x_rd16(REG_GPIOX_DIR) | (0x1 << 3));
    ft81x_wr16(REG_GPIOX, ft81x_rd16(REG_GPIOX) | (0x1 << 3));

    //// Start streaming
    ft81x_stream_start();

    //// Configure media fifo
    ft81x_cmd_mediafifo(0x100000UL-0x40000UL, 0x40000UL);

    //// Trigger FT81x to read the command buffer
    ft81x_getfree(0);

    //// Finish streaming to command buffer
    ft81x_stream_stop();

    //// stop media fifo
    ft81x_wr32(REG_MEDIAFIFO_WRITE, 0);
          
    //// Start streaming
    ft81x_stream_start();

    // Define the bitmap we want to draw
    //ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
    //ft81x_cmd_swap();     // Set AUTO swap at end of display list
    ft81x_clear_color_rgb32(0x505050);  
    ft81x_clear();
    ft81x_color_rgb32(0xff0000);
    ft81x_bgcolor_rgb32(0xff0000);
    ft81x_fgcolor_rgb32(0x0000ff);

    //// Play the clip
    ft81x_cmd_playvideo(OPT_MEDIAFIFO | OPT_NOTEAR | OPT_SOUND);

    // Draw some text and a number display value of dial
    ft81x_cmd_text(740, 300, 30, OPT_CENTERY, "Hello World");

    //// Trigger FT81x to read the command buffer
    ft81x_getfree(0);

    //// Finish streaming to command buffer
    ft81x_stream_stop();

    //// Wait till the GPU is finished
    // This screen will not finish as it wants to consume fifo data
    // For now to avoid blocking in this tests we will not wait.
    // We could thread the fifo spooling and block here till the screen ends
    // say a button is pushed on the display.
    // ft81x_wait_finish();

    size_t size = 0;
    uint8_t buffer[32*32]; // chunk size ESP32 max is 32 bytes with no DMA

    do {
      //// Read our block from the disk
      size = fread(buffer, 1, sizeof(buffer), fin);

      if (size <= 0) break;

      //// Send the image to the media fifo  
      ft81x_cSPOOL_MF(buffer, size);

    } while(size);
    
    // Sleep
    vTaskDelay(100 / portTICK_PERIOD_MS);

    fclose(fin);
    stop_sdcard();
#endif

#if 1 // Live WEBCAM interface demo

    // initialize vspi access mutex for shared access to the FT81X
    s_vspi_mutex = xSemaphoreCreateMutex();

   
    // Start mjpeg stream for CAM1
    mjpegcam_init();
    mjpegcam_start();
    
    // wakeup the display and set brightness
    ft81x_wake(22);

    //// Start streaming
    ft81x_stream_start();

    //// Configure media fifo
    ft81x_cmd_mediafifo(0x100000UL-0x40000UL, 0x40000UL);

    //// Trigger FT81x to read the command buffer
    ft81x_getfree(0);

    //// Finish streaming to command buffer
    ft81x_stream_stop();

    //// stop media fifo
    ft81x_wr32(REG_MEDIAFIFO_WRITE, 0);

    do {
      xSemaphoreTake(s_vspi_mutex, portMAX_DELAY);

      //// let everyone know we are ready for media fifo spooling to start
      mjpeg_spool_ready = 1;
      
      // download the display touch memory into ft81x_touch
      // Start streaming
      ft81x_get_touch_inputs();
      
      ft81x_stream_start();

      // Define the bitmap we want to draw
      ft81x_cmd_dlstart();  // Set REG_CMD_DL when done
      ft81x_cmd_swap();     // Set AUTO swap at end of display list
      ft81x_clear_color_rgb32(0xfdfdfd);
      ft81x_clear();
      ft81x_color_rgb32(0x0f0f0f);
      ft81x_bgcolor_rgb32(0x000000);
      ft81x_fgcolor_rgb32(0x0000ff);

      // Draw some text
      ft81x_cmd_text(650, 25, 30, OPT_CENTERY, "CAMERAS");
      ft81x_cmd_text(660, 473, 21, OPT_CENTERY, "AlarmDecoder.com");

      // Draw a button
      //// Turn on tagging
      ft81x_tag_mask(1);

      ft81x_tag(3); // tag the button #3
      ft81x_cmd_track(650, 50, 140, 50, 3); // track touches to the tag
      ft81x_cmd_button(650, 50, 140, 50, 31, 0, "CAM1");

      ft81x_tag(4); // tag the button #3
      ft81x_cmd_track(650, 110, 140, 50, 3); // track touches to the tag
      ft81x_cmd_button(650, 110, 140, 50, 31, 0, "CAM2");

      ft81x_tag(5); // tag the button #3
      ft81x_cmd_track(650, 170, 140, 50, 3); // track touches to the tag
      ft81x_cmd_button(650, 170, 140, 50, 31, 0, "CAM3");

      // Draw the image
      uint16_t pw = 640, ph = 480, st = pw * 2;
      ft81x_bitmap_source(0);
      ft81x_bitmap_layout(RGB565, st & 0x3ff, ph & 0x1ff);
      ft81x_bitmap_layout_h(st >> 10, ph >> 9);
          
      ft81x_bitmap_size(NEAREST, BORDER, BORDER, pw & 0x1ff, ph & 0x1ff);
      ft81x_bitmap_size_h(pw >> 9, ph >> 9);
      ft81x_color_rgb32(0xffffff);
      ft81x_begin(BITMAPS);
      ft81x_vertex2ii(0, 0, 0, 0);

      // Draw ON/OFF based upon touch
      if (ft81x_ctouch.tag0) {
        ft81x_color_rgb32(0xff0000);
        ft81x_bgcolor_rgb32(0xff0000);
        if(ft81x_ctouch.tag0 == 3)
          ft81x_cmd_text(280, 210, 31, OPT_CENTERY, "CAM1");
        if(ft81x_ctouch.tag0 == 4)
          ft81x_cmd_text(280, 210, 31, OPT_CENTERY, "CAM2");
        if(ft81x_ctouch.tag0 == 5)
          ft81x_cmd_text(280, 210, 31, OPT_CENTERY, "CAM3");
          break;
      }
      

      ft81x_display();

      //// Trigger FT81x to read the command buffer
      ft81x_getfree(0);

      //// Finish streaming to command buffer
      ft81x_stream_stop();

      ft81x_wait_finish();

      xSemaphoreGive(s_vspi_mutex);

      // Sleep 25fps is fast enough.
      vTaskDelay(40 / portTICK_PERIOD_MS);
      
    } while(1);
  #endif

    // stop the stream
    mjpegcam_stop();

    // stop the cam stream thread
    mjpegcam_deinit();

}
