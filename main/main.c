
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "role_config.h"
#include "wifi_comm.h"
#include "mqtt_comm.h"

#if DEVICE_ROLE == DEVICE_ROLE_BASE
#include "uart_gnss.h"
#elif DEVICE_ROLE == DEVICE_ROLE_ROVER
#include "rover_espnow_receiver.h"
#endif

// WiFi credentials
#define WIFI_SSID "CoffeeLiver"
#define WIFI_PASS "TinyBeauty"

// MQTT broker settings
#define MQTT_BROKER "mqtt://13.214.212.87:1883"
#define MQTT_USER "mqtt"
#define MQTT_PASS "ICPHmqtt!"
#define MQTT_TOPIC "rtk/corrections"

void app_main(void)
{
    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    esp_event_loop_create_default();

#if DEVICE_ROLE == DEVICE_ROLE_BASE
    printf("=== RTK BASE STATION (MQTT) ===\n");
    
    // Connect to WiFi
    wifi_init_sta(WIFI_SSID, WIFI_PASS);
    
    // Initialize MQTT
    mqtt_comm_init(MQTT_BROKER, MQTT_USER, MQTT_PASS, "base_station", MQTT_TOPIC);
    
    // Initialize UART to ZED-F9P
    uart_gnss_init();
    
    // Wait for MQTT connection
    printf("Waiting for MQTT connection...\n");
    while (!mqtt_is_connected()) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("MQTT Connected! Publishing RTCM corrections...\n");

    uint8_t data[512];
    int rtcm_seen = 0;
    int publish_count = 0;
    
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
            
            // Publish to MQTT
            mqtt_comm_publish(data, len);
            publish_count++;
            
            if (publish_count % 50 == 0) {
                printf("[BASE] Published %d packets (%d bytes each)\n", publish_count, len);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

#elif DEVICE_ROLE == DEVICE_ROLE_ROVER
    printf("=== RTK ROVER (MQTT) ===\n");
    
    // Connect to WiFi
    wifi_init_sta(WIFI_SSID, WIFI_PASS);
    
    // Initialize MQTT
    mqtt_comm_init(MQTT_BROKER, MQTT_USER, MQTT_PASS, "rover_station", MQTT_TOPIC);
    
    // Initialize UART to ZED-F9P (using rover functions)
    rover_espnow_receiver_init();
    
    // Wait for MQTT connection
    printf("Waiting for MQTT connection...\n");
    while (!mqtt_is_connected()) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("MQTT Connected! Receiving RTCM corrections...\n");

    static uint8_t gnss_buf[2048]; // Static to avoid stack overflow
    uint8_t mqtt_buf[512];
    size_t gnss_len = 0;
    int rtcm_count = 0;
    int loop_count = 0;
    
    while (1) {
        // Process all available RTCM messages from MQTT queue
        int processed = 0;
        int len;
        while ((len = mqtt_comm_receive(mqtt_buf, sizeof(mqtt_buf))) > 0) {
            rover_forward_to_gnss(mqtt_buf, len);
            rtcm_count++;
            processed++;
            if (processed >= 20) break;  // Process max 20 per loop to avoid starving other tasks
        }
        
        if (rtcm_count > 0 && rtcm_count % 10 == 0 && processed > 0) {
            printf("[ROVER] RTCM packets received: %d (batch: %d)\n", rtcm_count, processed);
        } else if (loop_count % 500 == 0) {
            // Every 5 seconds, report if no data
            printf("[ROVER] Waiting for RTCM... (total received: %d)\n", rtcm_count);
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