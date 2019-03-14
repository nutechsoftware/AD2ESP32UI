/**
 *  @file    mywifi.c
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

// std
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

// esp-idf general
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_err.h>

// drivers

// components
/// esp-idf specific
#include <esp_wifi.h>
#include <esp_http_client.h>

/// private
#include <multipart_parser.h>
#include <mywifi.h>
#include <ft81x.h>

/**
 * Constants/Statics/Globals
 */
 
// Debug tag
static const char *TAG = "MYWIFI";

// WIFI connection state vars
int s_retry_num = 0;

// EVENT group bits
const int WIFI_CONNECTED_BIT = BIT0;

// Streaming jpeg image ready flag
uint8_t mjpeg_spool_ready = 0;
uint16_t mjpeg_last_size = 0;

// mjpeg stream collector task handle
xTaskHandle mjpeg_client_handle = NULL;    

// FreeRTOS event group for signaling ex. connect/disconnect
EventGroupHandle_t s_wifi_event_group;

// VSPI access control
extern SemaphoreHandle_t s_vspi_mutex;

// FT81x state vars for debugging
extern uint32_t mf_wp;

// private prototypes
//// WIFI CONNECTION FUNCTIONS 
static esp_err_t mywifi_event_handler(void *ctx, system_event_t *event);
void app_wifi_wait_connected();

//// CAM STREAM FUNCTIONS
static void mjpeg_client_task(void *p);
int read_header_name(multipart_parser* p, const char *at, size_t length);
int read_header_value(multipart_parser* p, const char *at, size_t length);
int read_part_data(multipart_parser* p, const char *at, size_t length);
int read_part_data_end(multipart_parser* p);
esp_err_t _http_event_handler(esp_http_client_event_t *evt);
 
/**
 * Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem
 * The PEM file was extracted from the output of this command:
 * openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null
 * The CA root cert is the last cert given in the chain of certs.
 * To embed it in the app binary, the PEM file is named
 * in the component.mk COMPONENT_EMBED_TXTFILES variable.
 */
extern const char howsmyssl_com_root_cert_pem_start[] asm("_binary_howsmyssl_com_root_cert_pem_start");
extern const char howsmyssl_com_root_cert_pem_end[]   asm("_binary_howsmyssl_com_root_cert_pem_end");


/***************************
 * WIFI CONNECTION FUNCTIONS
 */
 
/**
 * Initialize the wifi driver
 */
bool mywifi_init()
{
    ESP_LOGI(TAG, "ESP32 Wifi driver init");
      
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(mywifi_event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = AD2ESP32UI_WIFI_SSID,
            .password = AD2ESP32UI_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
 
    return true;
}

/**
 * Connect to AP
 */
bool mywifi_start() 
{
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "connect to ap SSID:%s",
             AD2ESP32UI_WIFI_SSID);
    return true;             
}

/**
 * Disconnect from AP
 */
bool mywifi_stop() 
{
    ESP_ERROR_CHECK(esp_wifi_stop() ); 
    ESP_LOGI(TAG, "disconnect from SSID:%s",
             AD2ESP32UI_WIFI_SSID);
    return true;             
}

/**
 * Close the driver
 */
bool mywifi_deinit()
{
    return true;
}

/**
 * Wait blocking till connected letting other taskss run.
 */
void app_wifi_wait_connected()
{
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
}

/**
 * Event processing for wifi state
 */
static esp_err_t mywifi_event_handler(void *ctx, system_event_t *event)
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
            if (s_retry_num < AD2ESP32UI_MAXIMUM_RETRY) {
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


/***************************
 * CAM STREAM FUNCTIONS
 */

/**
 * Initialize the mjpeg stream collector task
 */
bool mjpegcam_init() 
{
    return true;
}

/**
 * Start the task
 */
bool mjpegcam_start()
{
    esp_err_t ret;

    // Start mjpeg socket client thread

    ret = xTaskCreate(mjpeg_client_task,
                      MJPEG_CLIENT_TASK_NAME,
                      MJPEG_CLIENT_TASK_STACK,
                      NULL,
                      MJPEG_CLIENT_TASK_PRIORITY,
                      &mjpeg_client_handle);

    if (ret != pdPASS)  {
        ESP_LOGI(TAG, "create thread %s failed", MJPEG_CLIENT_TASK_NAME);
    }
    
    return true;
}


/**
 * Stop the task
 */
bool mjpegcam_stop()
{
    ESP_LOGI(TAG, "STOP");  
    
    // if a processing task is running stop it.
    if (mjpeg_client_handle!= NULL) {
        vTaskDelete(mjpeg_client_handle);
        mjpeg_client_handle = NULL;
    }

    return true;
}

/**
 * Close the driver
 */
bool mjpegcam_deinit()
{
    ESP_LOGI(TAG, "DEINIT");

    return true;
}

/**
* Attempt to stay connected to a url
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
        mjpeg_client_handle = NULL; //not synced with mjpegcam_stop()
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
        if (read_len <= 0) {
            done = 1;
            break;
        }
        // parse using multipart parser
        multipart_parser_execute(parser, buffer, read_len);
    } while(!done);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(buffer);

    // free the multipart parser
    multipart_parser_free(parser);

    ESP_LOGI(TAG, "mjpeg_client_task done");
    mjpeg_client_handle = NULL; //not synced with mjpegcam_stop()
    vTaskDelete(NULL);
}

/**
 * http request event state logic
 */
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

/**
 * Callback from multipart_parser. 
 * #1. Receive header names from parser
 */
int read_header_name(multipart_parser* p, const char *at, size_t length)
{
    #if 0
    //ESP_LOGI(TAG, "HNAME: %.*s: ", length, at);
    printf("HNAME: %.*s: ", length, at);
    #endif   
    return 0;
}

/**
 * Callback from multipart_parser. 
 * #2. Receive header values from parser
 */
int read_header_value(multipart_parser* p, const char *at, size_t length)
{
    #if 0
    //ESP_LOGI(TAG, "HVAL: %.*s", length, at);
    printf("HVAL: %.*s\n", length, at);
    #endif  
    return 0;
}

/**
 * Callback from multipart_parser
 * End of a part
 */
int read_part_data_end(multipart_parser* p) {
  
    // take ownership of the VSPI bus
    xSemaphoreTake(s_vspi_mutex, portMAX_DELAY);

    // We have loaded the entire image into MEDIA FIFO 
    // but we must be sure our data is aligned to 32 bits.
    // It is not clear if this is needed on MEDIA_FIFO but
    // is needed when loading images via the command buffer.
    int8_t align = 4 - (mjpeg_last_size & 0x3);
    if (align & 0x3) {
        uint8_t dummy[4] = {0x00, 0x00, 0x00, 0x00};
        ft81x_cSPOOL_MF(dummy, align);       
    }

    // Start streaming
    ft81x_stream_start();

    // USE MEDIA_FIFO
    // Load the image at address 0 our streaming image address
    ft81x_cmd_loadimage(0, OPT_RGB565 | OPT_NODL | OPT_MEDIAFIFO);

    // Get the decompressed image properties
    uint32_t imgptr, widthptr, heightptr;   
    ft81x_cmd_getprops(&imgptr, &widthptr, &heightptr);

    // Trigger FT81x to read the command buffer
    ft81x_getfree(0);

    // Finish streaming to command buffer
    ft81x_stream_stop();

    ft81x_wait_finish();

    // FT81x seems to read in 32 bit chunks only so it will read past my write pointer.
    mf_wp = ft81x_rd32(REG_MEDIAFIFO_READ);

    #if 0
    // SPI Debugging
    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_16, 0);
    #endif

    // Dump results
    uint32_t img = ft81x_rd32(imgptr); // pointer to end of image and start of next free memory
    uint32_t width = ft81x_rd32(widthptr);
    uint32_t height = ft81x_rd32(heightptr);
    #if 1
    ESP_LOGW(TAG, "loadimage wp:0x%08x ptr:0x%08x width: %d height: %d size: %d", mf_wp, img, width, height, mjpeg_last_size);
    #endif

    mjpeg_last_size = 0;

    // release the VSPI bus
    xSemaphoreGive(s_vspi_mutex);

    #if 1 // RACE COND TEST
    // Sleep
    vTaskDelay(10 / portTICK_PERIOD_MS);
    #endif


    return 0;
}


/**
 * Callback from multipart_parser.
 * When a block of data for the current part is ready.
 *
 * TODO: multipart_parser has some issues. It will send out single \n
 * bytes to be processed. An improvement to multipart_parser would be to 
 * better detect the boundary start and push the \n back onto the stack. 
 */
int read_part_data(multipart_parser* p, const char *at, size_t length)
{
  
    // just to be safe.
    if(!length)
        return 0;

    // The the FIFO stream is not being consumed we can block or crash the FT813
    if (mjpeg_spool_ready) {

        // Take owership of the VSPI bus to do a bulk write to MEDIA FIFO
        xSemaphoreTake(s_vspi_mutex, portMAX_DELAY);

        //// Send the image to the media fifo
        ft81x_cSPOOL_MF((uint8_t *)at, length);    

        mjpeg_last_size+=length;
        //ESP_LOGI(TAG, "httpread size: %d len: %d)",mjpeg_last_size, length);

        xSemaphoreGive(s_vspi_mutex);    
    }


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
