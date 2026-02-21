#include "rover_wifi_receiver.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define WIFI_PORT 5000
#define UART_GNSS_PORT_NUM UART_NUM_1
#define UART_GNSS_TXD 17
#define UART_GNSS_RXD 16
#define UART_GNSS_BAUD_RATE 115200
#define UART_GNSS_BUF_SIZE 512

static const char *TAG = "ROVER_WIFI";
static int sock = -1;

void rover_wifi_receiver_init(const char *ssid, const char *password) {
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
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, 32);
    strncpy((char *)wifi_config.sta.password, password, 64);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();

    // Wait for connection (simple delay for demo)
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // Socket setup
    struct sockaddr_in local_addr;
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        return;
    }
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(WIFI_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr));

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

int rover_wifi_receive(uint8_t *data, size_t max_len) {
    if (sock < 0) return 0;
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    int len = recvfrom(sock, data, max_len, MSG_DONTWAIT, (struct sockaddr *)&from, &fromlen);
    return len > 0 ? len : 0;
}

void rover_forward_to_gnss(const uint8_t *data, size_t len) {
    uart_write_bytes(UART_GNSS_PORT_NUM, (const char *)data, len);
}
