
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "role_config.h"

#if DEVICE_ROLE == DEVICE_ROLE_BASE
#include "uart_gnss.h"
#include "espnow_comm.h"
#elif DEVICE_ROLE == DEVICE_ROLE_ROVER
#include "rover_espnow_receiver.h"
#endif

void app_main(void)
{
    // Initialize NVS (required for Wi-Fi/ESP-NOW)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if DEVICE_ROLE == DEVICE_ROLE_BASE
    printf("=== RTK BASE STATION ===\n");
    espnow_comm_init();
    uart_gnss_init();

    uint8_t data[512];
    int rtcm_seen = 0;
    
    while (1) {
        int len = uart_gnss_read(data, sizeof(data));
        if (len > 0) {
            // Check for RTCM (0xD3 = start of RTCM3)
            for (int i = 0; i < len; i++) {
                if (data[i] == 0xD3) {
                    if (!rtcm_seen) {
                        rtcm_seen = 1;
                        printf("\n*** SURVEY-IN COMPLETE - RTCM ACTIVE ***\n\n");
                    }
                    break;
                }
            }
            
            // Forward all data to rover immediately
            espnow_comm_send(data, len);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

#elif DEVICE_ROLE == DEVICE_ROLE_ROVER
    printf("=== RTK ROVER ===\n");
    rover_espnow_receiver_init();

    static uint8_t gnss_buf[2048]; // Static to avoid stack overflow
    uint8_t buf[256];
    size_t gnss_len = 0;
    int rtcm_count = 0;
    int loop_count = 0;
    
    while (1) {
        // Receive RTCM from base and forward to ZED-F9P
        int len = rover_espnow_receive(buf, sizeof(buf));
        if (len > 0) {
            rover_forward_to_gnss(buf, len);
            rtcm_count++;
            if (rtcm_count % 50 == 0) {
                printf("[ROVER] RTCM: %d packets\n", rtcm_count);
            }
        }
        
        // Read GPS data from ZED-F9P
        int read_len = rover_uart_read(gnss_buf + gnss_len, sizeof(gnss_buf) - gnss_len);
        if (read_len > 0) {
            gnss_len += read_len;
            
            // Shift buffer if getting full
            if (gnss_len > sizeof(gnss_buf) - 256) {
                memmove(gnss_buf, gnss_buf + 1024, gnss_len - 1024);
                gnss_len -= 1024;
            }
        }
        
        // Display GPS info every ~2 seconds
        if (++loop_count >= 200) {
            loop_count = 0;
            rover_display_gga(gnss_buf, gnss_len);
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
#endif
}