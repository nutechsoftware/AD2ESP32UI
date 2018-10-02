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

#include <stdio.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/dirent.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "soc/gpio_struct.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/i2s.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include <ft81x.h>
#include <pdmmic.h>
#include <multipart_parser.h>
/*
 * Defines
 */ 

/*
 * Constants/Statics
 */ 
// Debug tag
static const char* TAG = "AD2TOUCH";
extern struct ft81x_ctouch_t ft81x_ctouch;
extern struct ft81x_touch_t ft81x_touch;
extern uint32_t mf_wp;

//WIFI 
static int s_retry_num = 0;

// Streaming jpeg image ready flag
static uint8_t mjpeg_spool_ready = 0;
static SemaphoreHandle_t s_ipc_mutex;
static uint16_t debug_size = 0;

/* The event group allows multiple bits for each event, but we only care about one event 
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* WIFI settings */
#define EXAMPLE_ESP_WIFI_SSID         CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS         CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY     10
#define MAX_HTTP_RECV_BUFFER          512

/* WEBCAM MJPEG collector task */
#define MJPEG_CLIENT_TASK_NAME        "mjpeg_client_task"
#define MJPEG_CLIENT_TASK_NAME_STACK  10240

/*
 * Types
 */

 
/*
 * Prototypes
 */ 
// Enable GPIO pin for bit bang debugging good for timing tests :c)
bool enable_gpio_debug_pin();

// restart the esp32
void restart_esp32();

// tests sdcard using spi interface HSPI
void test_spi_sdcard();
bool start_sdcard(sdmmc_card_t *card);
void stop_sdcard();
// tests spooling video from SD card to FT813
void test_video();

// WiFi/Network
void wifi_init_sta();
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
void app_wifi_wait_connected();

/*
 * Functions
 */
 
 int read_header_name(multipart_parser* p, const char *at, size_t length)
{
#if 0
   //ESP_LOGI(TAG, "HNAME: %.*s: ", length, at);
   printf("HNAME: %.*s: ", length, at);
#endif   
   return 0;
}

int read_header_value(multipart_parser* p, const char *at, size_t length)
{
#if 0
   //ESP_LOGI(TAG, "HVAL: %.*s", length, at);
   printf("HVAL: %.*s\n", length, at);
#endif  
   return 0;
}

static uint32_t camimagesize = 0;
int read_part_data_end(multipart_parser* p) {
  
     xSemaphoreTake(s_ipc_mutex, portMAX_DELAY);

     // We have loaded the entire image into MEDIA FIFO 
     // but we must be sure our data is aligned to 32 bits.
     // It is not clear if this is needed on MEDIA_FIFO but
     // is needed when loading images via the command buffer.
     int8_t align = 4 - (camimagesize & 0x3);
     if (align & 0x3) {
       uint8_t dummy[4] = {0x00, 0x00, 0x00, 0x00};
       ft81x_cSPOOL_MF(dummy, align);       
     }

     // Load the OFF image
     //// Start streaming
     ft81x_stream_start();

     //// USE MEDIA_FIFO
     //// Load the image at address 0 our streaming image address
     ft81x_cmd_loadimage(0, OPT_RGB565 | OPT_NODL | OPT_MEDIAFIFO);

     //// Get the decompressed image properties
     uint32_t imgptr, widthptr, heightptr;   
     ft81x_cmd_getprops(&imgptr, &widthptr, &heightptr);

     //// Trigger FT81x to read the command buffer
     ft81x_getfree(0);

     //// Finish streaming to command buffer
     ft81x_stream_stop();

     //printf("WAIT.");
     ft81x_wait_finish();
#if 0 // RACE COND TEST
     vTaskDelay(100 / portTICK_PERIOD_MS);
#endif
     // FT81x seems to read in 32 bit chunks only so it will read past my write pointer.
     mf_wp = ft81x_rd32(REG_MEDIAFIFO_READ);

#if 1
     // SPI Debugging
     gpio_set_level(GPIO_NUM_16, 1);
     gpio_set_level(GPIO_NUM_16, 0);
#endif

     //// Dump results
     uint32_t img = ft81x_rd32(imgptr); // pointer to end of image and start of next free memory
     uint32_t width = ft81x_rd32(widthptr);
     uint32_t height = ft81x_rd32(heightptr);
#if 1
     ESP_LOGW(TAG, "loadimage wp:0x%08x ptr:0x%08x width: %d height: %d size: %d", mf_wp, img, width, height, debug_size);
     //printf("loadimage wp:0x%06x ptr:0x%06x width: %d height: %d\n", mf_wp, img, width, height);
#endif

     debug_size = 0;
     
     camimagesize = 0;

     xSemaphoreGive(s_ipc_mutex);

#if 1 // RACE COND TEST
     // Sleep
     vTaskDelay(10 / portTICK_PERIOD_MS);
#endif


  return 0;
}

int read_part_data(multipart_parser* p, const char *at, size_t length)
{

  if(!length)
     return 0;

  xSemaphoreTake(s_ipc_mutex, portMAX_DELAY);

  if (mjpeg_spool_ready) {
    //// Send the image to the media fifo
    ft81x_cSPOOL_MF((uint8_t *)at, length);
    camimagesize+=length;
  }

  debug_size+=length;
  //ESP_LOGI(TAG, "httpread size: %d len: %d)",debug_size, length);

  xSemaphoreGive(s_ipc_mutex);

#if 0
   printf("VDATA %d: ", length);
   for (int i = 0; i < length; i++)
   {
      printf("%02x", at[i]);
   }
   printf("\n");
#endif

   return 0;
}

/*
 * mjpeg_client_task
 * Will attempt to stay connected to a url
 * and capture the multipart mjpeg stream
 * converting each frame into a MEDIA FIFO 
 * update to a pointer in the FT81X memory.
 */
static void mjpeg_client_task(void *p)
{
  ESP_LOGI(TAG, "mjpeg_client_task start");
  
  // setup the multipart parser
  multipart_parser_settings callbacks;
  memset(&callbacks, 0, sizeof(multipart_parser_settings));

  callbacks.on_header_field = read_header_name;
  callbacks.on_header_value = read_header_value;
  callbacks.on_part_data = read_part_data;
  callbacks.on_part_data_end = read_part_data_end;

  char *buffer = malloc(MAX_HTTP_RECV_BUFFER + 1);

  esp_http_client_config_t config = {
      .url = CONFIG_AD2_CAM1_URL,
      .event_handler = _http_event_handler
  };
  
  // Wait till the connected flag is set
  app_wifi_wait_connected();
  
  esp_http_client_handle_t client = esp_http_client_init(&config);
  esp_err_t err;
  if ((err = esp_http_client_open(client, 0)) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
      free(buffer);
      vTaskDelete(NULL);      
      return;
  }
  
  ESP_LOGI(TAG, "mjpeg_client_task connected");
  multipart_parser* parser = multipart_parser_init("--BoundaryString", &callbacks);
 
  int content_length =  esp_http_client_fetch_headers(client);
  ESP_LOGI(TAG, "HTTP Stream reader Status = %d, content_length = %d cl = %d",
                  esp_http_client_get_status_code(client),
                  esp_http_client_get_content_length(client), content_length);

  uint8_t done = 0;
  do {
    int read_len = esp_http_client_read(client, buffer, MAX_HTTP_RECV_BUFFER);
    // parse using multipart parser
    multipart_parser_execute(parser, buffer, read_len);
  } while(!done);
  


  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  free(buffer);

  // free the multipart parser
  multipart_parser_free(parser);

  ESP_LOGI(TAG, "mjpeg_client_task done");
  vTaskDelete(NULL);
  return ;  
}

/*
 * FreeRTOS main() entry point
 *
 */
void app_main()
{
    bool res;

    // Disable buffering on stdout
    setvbuf(stdout, NULL, _IONBF, 0);
    
    printf("AD2TS STARTING.\n");

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
    
#if 0
    // delay test before spi to allow power on of device and usb connect time
    ESP_LOGW(TAG, "initGPU: delay");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
#endif

    // Initialize our SPI driver on ESP32 VSPI pins for the FT81X
    res = ft81x_initSPI();
    if (!res) {
      printf("ESP32 SPI init failed\n");
      restart_esp32();
    } else {
      printf("ESP32 SPI init success\n");
    }

#if 1 // Test GPU
    // Initialize the FT81X GPU
    res = ft81x_initGPU();
    if (!res) {
      printf("FT81X init failed\n");
      restart_esp32();
    } else {
      printf("FT81X init success ID 0x%04x\n", ft81x_chip_id);
    }
#endif

    // Test SD CARD
    //test_spi_sdcard();
    test_video();

    // Testing done
    restart_esp32();
}

/*
 * Restart ESP32 chip
 */
void restart_esp32() {
  for (int i = 4; i >= 0; i--) {
      printf("Restarting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_restart();
}


/* 
 * Enable an output pin for bit banging debug data
 */
bool enable_gpio_debug_pin() {
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

bool start_sdcard(sdmmc_card_t* card) {
  #define PIN_NUM_MISO 12 // GREEN   D0 | D0 
  #define PIN_NUM_MOSI 13 // YELLOW CMD | DI
  #define PIN_NUM_CLK  14 // PURPLE CLK
  #define PIN_NUM_CS   15 // WHITE   D3 | CS

  ESP_LOGI(TAG, "Initializing SD card using SPI");

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  slot_config.gpio_miso = PIN_NUM_MISO;
  slot_config.gpio_mosi = PIN_NUM_MOSI;
  slot_config.gpio_sck  = PIN_NUM_CLK;
  slot_config.gpio_cs   = PIN_NUM_CS;
  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.

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

void stop_sdcard() {
  // All done, unmount partition and disable SDMMC or SPI peripheral
  esp_vfs_fat_sdmmc_unmount();
  ESP_LOGI(TAG, "Card unmounted");  
}

void test_spi_sdcard() {

  #define PIN_NUM_MISO 12 // GREEN   D0
  #define PIN_NUM_MOSI 13 // YELLOW CMD
  #define PIN_NUM_CLK  14 // PURPLE CLK
  #define PIN_NUM_CS   15 // WHITE   D3

  ESP_LOGI(TAG, "Initializing SD card using SPI");

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  
  slot_config.gpio_miso = PIN_NUM_MISO;
  slot_config.gpio_mosi = PIN_NUM_MOSI;
  slot_config.gpio_sck  = PIN_NUM_CLK;
  slot_config.gpio_cs   = PIN_NUM_CS;
  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.

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
  sdmmc_card_t* card;

#if 0
  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);
#endif

  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

#if 0
  // SPI Debugging
  gpio_set_level(GPIO_NUM_16, 1);
  gpio_set_level(GPIO_NUM_16, 0);
#endif

  if (ret != ESP_OK) {
     if (ret == ESP_FAIL) {
         ESP_LOGE(TAG, "Failed to mount filesystem. "
             "If you want the card to be formatted, set format_if_mount_failed = true.");
     } else {
         ESP_LOGE(TAG, "Failed to initialize the card (%s). "
             "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
     }
     return;
  }

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, card);

  // Use POSIX and C standard library functions to work with files.
  // First create a file.
  ESP_LOGI(TAG, "Opening file");
  FILE* f = fopen("/sdcard/hello.txt", "w");
  if (f == NULL) {
     ESP_LOGE(TAG, "Failed to open file for writing");
     return;
  }
  fprintf(f, "Hello %s!\n", card->cid.name);
  fclose(f);
  ESP_LOGI(TAG, "File written");

  // Check if destination file exists before renaming
  struct stat st;
  if (stat("/sdcard/foo.txt", &st) == 0) {
     // Delete it if it exists
     unlink("/sdcard/foo.txt");
  }

  // Rename original file
  ESP_LOGI(TAG, "Renaming file");
  if (rename("/sdcard/hello.txt", "/sdcard/foo.txt") != 0) {
     ESP_LOGE(TAG, "Rename failed");
     return;
  }

  // Open renamed file for reading
  ESP_LOGI(TAG, "Reading file");
  f = fopen("/sdcard/foo.txt", "r");
  if (f == NULL) {
     ESP_LOGE(TAG, "Failed to open file for reading");
     return;
  }
  char line[64];
  fgets(line, sizeof(line), f);
  fclose(f);
  // strip newline
  char* pos = strchr(line, '\n');
  if (pos) {
     *pos = '\0';
  }
  ESP_LOGI(TAG, "Read from file: '%s'", line);

  // List files in path
  ESP_LOGI(TAG, "Reading directory listing");
  DIR *dp;
  struct dirent *ep;     
  dp = opendir ("/sdcard/DCIM");

  if (dp != NULL)
  {
    while ((ep = readdir (dp)))
      ESP_LOGI(TAG, "File : %s", ep->d_name);

    (void) closedir (dp);
  }
  else
     ESP_LOGE(TAG, "Couldn't open the directory");
 
  
  // All done, unmount partition and disable SDMMC or SPI peripheral
  esp_vfs_fat_sdmmc_unmount();
  ESP_LOGI(TAG, "Card unmounted");
}

void test_video() {

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
  esp_err_t ret;

#if 0
  // start the microphone listening thread
  pdmmic_init();
  pdmmic_start();
#endif
  
  // Start the WIFI client
  wifi_init_sta();
  
    
  // Start mjpeg socket client thread
  xTaskHandle mjpeg_client_handle;    
  ret = xTaskCreate(mjpeg_client_task,
                    MJPEG_CLIENT_TASK_NAME,
                    MJPEG_CLIENT_TASK_NAME_STACK,
                    NULL,
                    8,
                    &mjpeg_client_handle);

  if (ret != pdPASS)  {
      ESP_LOGI(TAG, "create thread %s failed", MJPEG_CLIENT_TASK_NAME);
  }

  // Start our SD Card
  sdmmc_card_t card;  
  start_sdcard(&card);

  /* initialize mutex */
  s_ipc_mutex = xSemaphoreCreateMutex();

  ft81x_wr(REG_PWM_DUTY, 22);

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
    xSemaphoreTake(s_ipc_mutex, portMAX_DELAY);

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
    }
    

    ft81x_display();

    //// Trigger FT81x to read the command buffer
    ft81x_getfree(0);

    //// Finish streaming to command buffer
    ft81x_stream_stop();

    ft81x_wait_finish();

    xSemaphoreGive(s_ipc_mutex);

#if 1 // RACE COND TEST 
    // Sleep
    vTaskDelay(200 / portTICK_PERIOD_MS);
#endif
    
  } while(1);
#endif

}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
                esp_wifi_connect();
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                s_retry_num++;
                ESP_LOGI(TAG,"retry to connect to the AP");
            }
            ESP_LOGI(TAG,"connect to the AP fail\n");
            break;
        }
    default:
        break;
    }
    return ESP_OK;
}

void wifi_init_sta()
{
    ESP_LOGI(TAG, "STARTING WIFI CLIENT");
      
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}


/* Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem
   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null
   The CA root cert is the last cert given in the chain of certs.
   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

void app_wifi_wait_connected()
{
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
}
