#include "mqtt_comm.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include <string.h>

static const char *TAG = "MQTT_COMM";
static esp_mqtt_client_handle_t client = NULL;
static char mqtt_topic[128] = {0};
static uint8_t rx_buffer[512];
static volatile int rx_len = 0;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_DATA:
            if (event->data_len > 0 && event->data_len < sizeof(rx_buffer)) {
                memcpy(rx_buffer, event->data, event->data_len);
                rx_len = event->data_len;
            }
            break;
        default:
            break;
    }
}

void mqtt_comm_init(const char *broker_url, const char *client_id, const char *topic) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_event_loop_create_default();
    strncpy(mqtt_topic, topic, sizeof(mqtt_topic) - 1);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = broker_url,
            },
        },
        .credentials = {
            .client_id = client_id,
        },
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    esp_mqtt_client_subscribe(client, mqtt_topic, 0);
}

void mqtt_comm_publish(const uint8_t *data, size_t len) {
    if (client) {
        esp_mqtt_client_publish(client, mqtt_topic, (const char *)data, len, 0, 0);
    }
}

int mqtt_comm_subscribe(uint8_t *data, size_t max_len) {
    if (rx_len > 0) {
        int len = rx_len > max_len ? max_len : rx_len;
        memcpy(data, rx_buffer, len);
        rx_len = 0;
        return len;
    }
    return 0;
}
