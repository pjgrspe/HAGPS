#include "rover_espnow_receiver.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include <string.h>

#define UART_GNSS_PORT_NUM UART_NUM_1
#define UART_GNSS_TXD 17
#define UART_GNSS_RXD 16
#define UART_GNSS_BAUD_RATE 115200
#define UART_GNSS_BUF_SIZE 512
#define ESPNOW_MAX_SIZE 250

static const char *TAG = "ROVER_ESPNOW";
static uint8_t rx_buffer[ESPNOW_MAX_SIZE];
static volatile int rx_len = 0;

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len > 0 && len <= ESPNOW_MAX_SIZE) {
        memcpy(rx_buffer, data, len);
        rx_len = len;
    }
}

void rover_espnow_receiver_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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
    esp_now_register_recv_cb(espnow_recv_cb);

    // UART GNSS setup
    const uart_config_t uart_config = {
        .baud_rate = UART_GNSS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_GNSS_PORT_NUM, &uart_config);
    uart_set_pin(UART_GNSS_PORT_NUM, UART_GNSS_TXD, UART_GNSS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_GNSS_PORT_NUM, UART_GNSS_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int rover_espnow_receive(uint8_t *data, size_t max_len) {
    if (rx_len > 0) {
        int len = rx_len > max_len ? max_len : rx_len;
        memcpy(data, rx_buffer, len);
        rx_len = 0;
        return len;
    }
    return 0;
}

void rover_forward_to_gnss(const uint8_t *data, size_t len) {
    uart_write_bytes(UART_GNSS_PORT_NUM, (const char *)data, len);
}
