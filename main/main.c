
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_gnss.h"
#include "espnow_comm.h"
#include "nvs_flash.h"



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

            // Print as ASCII for NMEA/debug
            printf("Read %d bytes (ascii): ", len);
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                if (c >= 32 && c <= 126) {
                    putchar(c);
                } else if (c == '\r' || c == '\n') {
                    putchar(c);
                } else {
                    putchar('.');
                }
            }
            printf("\n");

            espnow_comm_send(data, len);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}