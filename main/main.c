
#include <stdio.h>
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

#define ESP_NOW_MAX_SIZE 250

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

    uint8_t data[ESP_NOW_MAX_SIZE];
    while (1) {
        int len = uart_gnss_read(data, sizeof(data));
        if (len > 0) {
            printf("Read %d bytes (hex): ", len);
            for (int i = 0; i < len; i++) {
                printf("%02X ", data[i]);
            }
            printf("\n");
            espnow_comm_send(data, len);
        } else {
            static int survey_counter = 0;
            if (survey_counter % 20 == 0) {
                printf("No GNSS data yet. Survey-In in progress...\n");
            }
            survey_counter++;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

#elif DEVICE_ROLE == DEVICE_ROLE_ROVER
    printf("=== RTK ROVER - MONITORING MODE ===\n");
    rover_espnow_receiver_init();

    uint8_t buf[ESP_NOW_MAX_SIZE];
    uint8_t gnss_buf[256];
    int fix_status = -1;
    const char *fix_names[] = {"Invalid", "GPS", "DGPS", "PPS", "RTK Fixed", "RTK Float"};
    int status_counter = 0;
    
    while (1) {
        // Forward ESP-NOW data to ZED-F9P
        int len = rover_espnow_receive(buf, sizeof(buf));
        if (len > 0) {
            printf("Received %d bytes over ESP-NOW, forwarding to GNSS...\n", len);
            rover_forward_to_gnss(buf, len);
        }
        
        // Read ZED-F9P output to monitor fix status
        int gnss_len = rover_uart_read(gnss_buf, sizeof(gnss_buf));
        if (gnss_len > 0) {
            int new_fix = rover_parse_gga_fix(gnss_buf, gnss_len);
            if (new_fix >= 0 && new_fix != fix_status) {
                fix_status = new_fix;
                printf("\n*** FIX STATUS CHANGED: %s ***\n", 
                       fix_status <= 5 ? fix_names[fix_status] : "Unknown");
            }
        }
        
        // Print status periodically
        if (++status_counter >= 100) {
            status_counter = 0;
            if (fix_status >= 0) {
                printf("Current fix: %s\n", fix_status <= 5 ? fix_names[fix_status] : "Unknown");
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
#endif
}