#include "rover_espnow_receiver.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include <string.h>

// Use UART2 (RX2/TX2 pins) - UART1 is used by USB connection
#define UART_GNSS_PORT_NUM UART_NUM_2
#define UART_GNSS_TXD 17
#define UART_GNSS_RXD 16
#define UART_GNSS_BAUD_RATE 115200
#define UART_GNSS_BUF_SIZE 512
#define ESPNOW_MAX_SIZE 250

static const char *TAG = "ROVER_ESPNOW";
static uint8_t rx_buffer[ESPNOW_MAX_SIZE];
static volatile int rx_len = 0;

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
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
    esp_err_t err;
    
    err = uart_param_config(UART_GNSS_PORT_NUM, &uart_config);
    ESP_LOGI(TAG, "uart_param_config: %s", esp_err_to_name(err));
    
    err = uart_set_pin(UART_GNSS_PORT_NUM, UART_GNSS_TXD, UART_GNSS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "uart_set_pin(TX=%d, RX=%d): %s", UART_GNSS_TXD, UART_GNSS_RXD, esp_err_to_name(err));
    
    err = uart_driver_install(UART_GNSS_PORT_NUM, UART_GNSS_BUF_SIZE * 2, UART_GNSS_BUF_SIZE * 2, 0, NULL, 0);
    ESP_LOGI(TAG, "uart_driver_install: %s", esp_err_to_name(err));
    
    // Test UART TX with a UBX command (poll firmware version)
    // UBX-MON-VER poll: B5 62 0A 04 00 00 0E 34
    // If ZED-F9P receives this, it will respond in u-center
    const uint8_t ubx_poll_ver[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
    int test_written = uart_write_bytes(UART_GNSS_PORT_NUM, (const char *)ubx_poll_ver, sizeof(ubx_poll_ver));
    ESP_LOGI(TAG, "UART test: Sent UBX-MON-VER poll (%d bytes). Check u-center packet console for response.", test_written);
    uart_wait_tx_done(UART_GNSS_PORT_NUM, pdMS_TO_TICKS(100));
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
    // RTCM3 messages start with 0xD3
    if (len > 0) {
        ESP_LOGI(TAG, "RTCM data header: 0x%02X (expect 0xD3 for RTCM3)", data[0]);
    }
    
    int written = uart_write_bytes(UART_GNSS_PORT_NUM, (const char *)data, len);
    ESP_LOGI(TAG, "UART wrote %d bytes (requested %d)", written, len);
    
    // Flush to ensure data is sent immediately
    uart_wait_tx_done(UART_GNSS_PORT_NUM, pdMS_TO_TICKS(100));
}

int rover_uart_read(uint8_t *data, size_t max_len) {
    return uart_read_bytes(UART_GNSS_PORT_NUM, data, max_len, pdMS_TO_TICKS(10));
}

// Parse NMEA GGA sentence to extract fix quality
// Fix quality: 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
int rover_parse_gga_fix(const uint8_t *data, size_t len) {
    // Look for $GNGGA or $GPGGA
    for (size_t i = 0; i < len - 50; i++) {
        if (data[i] == '$' && data[i+2] == 'G' && data[i+3] == 'G' && data[i+4] == 'A') {
            // Find 6th comma (fix quality field)
            int comma_count = 0;
            for (size_t j = i; j < i + 80 && j < len; j++) {
                if (data[j] == ',') {
                    comma_count++;
                    if (comma_count == 6 && j + 1 < len) {
                        return data[j + 1] - '0'; // Convert ASCII to int
                    }
                }
            }
        }
    }
    return -1; // Not found
}
