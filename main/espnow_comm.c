#include "espnow_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ESPNOW_COMM";
static uint8_t broadcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "ESP-NOW send status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void espnow_comm_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed");
        return;
    }
    esp_now_register_send_cb(espnow_send_cb);

    esp_now_peer_info_t peerInfo = {0};
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = 0;
    peerInfo.ifidx = ESP_IF_WIFI_STA;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ESP-NOW peer");
        return;
    }
}

void espnow_comm_send(const uint8_t *data, size_t len) {
    esp_err_t result = esp_now_send(broadcast_mac, data, len);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW send error: %d", result);
    }
}
