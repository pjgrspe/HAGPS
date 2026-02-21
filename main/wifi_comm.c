#include "wifi_comm.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define WIFI_SSID_MAXLEN 32
#define WIFI_PASS_MAXLEN 64
#define WIFI_PORT 5000

static const char *TAG = "WIFI_COMM";
static int sock = -1;
static struct sockaddr_in peer_addr;

void wifi_comm_init(const char *ssid, const char *password, int is_base) {
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
    strncpy((char *)wifi_config.sta.ssid, ssid, WIFI_SSID_MAXLEN);
    strncpy((char *)wifi_config.sta.password, password, WIFI_PASS_MAXLEN);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();

    // Wait for connection (simple delay for demo)
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // Socket setup
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        return;
    }

    memset(&peer_addr, 0, sizeof(peer_addr));
    peer_addr.sin_family = AF_INET;
    peer_addr.sin_port = htons(WIFI_PORT);
    // For demo: set peer IP to broadcast (change to rover/base IP as needed)
    peer_addr.sin_addr.s_addr = inet_addr("255.255.255.255");

    // Allow broadcast
    int broadcastEnable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
}

void wifi_comm_send(const uint8_t *data, size_t len) {
    if (sock < 0) return;
    sendto(sock, data, len, 0, (struct sockaddr *)&peer_addr, sizeof(peer_addr));
}

int wifi_comm_receive(uint8_t *data, size_t max_len) {
    if (sock < 0) return 0;
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    int len = recvfrom(sock, data, max_len, MSG_DONTWAIT, (struct sockaddr *)&from, &fromlen);
    return len > 0 ? len : 0;
}
