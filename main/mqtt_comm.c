#include "mqtt_comm.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>

#define MQTT_QUEUE_SIZE 10

typedef struct {
    uint8_t data[512];
    size_t len;
} mqtt_msg_t;

static const char *TAG = "MQTT_COMM";
static esp_mqtt_client_handle_t client = NULL;
static char mqtt_topic[128] = {0};
static QueueHandle_t mqtt_queue = NULL;
static volatile int connected = 0;
static volatile int total_received = 0;
static volatile int dropped = 0;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            connected = 1;
            esp_mqtt_client_subscribe(client, mqtt_topic, 1);  // QoS 1 for reliability
            ESP_LOGI(TAG, "Subscribed to topic: %s", mqtt_topic);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            connected = 0;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT Subscribed, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            // Only process if topic matches
            if (strncmp(event->topic, mqtt_topic, event->topic_len) == 0) {
                total_received++;
                // Try to queue the message
                if (event->data_len > 0 && event->data_len <= 512) {
                    mqtt_msg_t msg;
                    memcpy(msg.data, event->data, event->data_len);
                    msg.len = event->data_len;
                    
                    if (xQueueSend(mqtt_queue, &msg, 0) != pdTRUE) {
                        dropped++;
                        if (dropped % 20 == 0) {
                            ESP_LOGW(TAG, "Dropped %d packets (queue full)", dropped);
                        }
                    } else if (total_received % 100 == 0) {
                        ESP_LOGI(TAG, "MQTT received %d packets (dropped: %d)", total_received, dropped);
                    }
                } else {
                    dropped++;
                }
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error");
            break;
        default:
            break;
    }
}

void mqtt_comm_init(const char *broker_uri, const char *username, const char *password, const char *client_id, const char *topic) {
    strncpy(mqtt_topic, topic, sizeof(mqtt_topic) - 1);
    
    // Create queue for incoming MQTT messages
    mqtt_queue = xQueueCreate(MQTT_QUEUE_SIZE, sizeof(mqtt_msg_t));
    if (mqtt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create MQTT queue");
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = broker_uri,
            },
        },
        .credentials = {
            .username = username,
            .authentication = {
                .password = password,
            },
            .client_id = client_id,
        },
    };
    
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    
    ESP_LOGI(TAG, "MQTT client started, connecting to %s", broker_uri);
}

void mqtt_comm_publish(const uint8_t *data, size_t len) {
    if (client && connected) {
        esp_mqtt_client_publish(client, mqtt_topic, (const char *)data, len, 0, 0);
    }
}

int mqtt_comm_receive(uint8_t *data, size_t max_len) {
    mqtt_msg_t msg;
    if (xQueueReceive(mqtt_queue, &msg, 0) == pdTRUE) {
        size_t copy_len = msg.len > max_len ? max_len : msg.len;
        memcpy(data, msg.data, copy_len);
        return copy_len;
    }
    return 0;
}

int mqtt_is_connected(void) {
    return connected;
}
