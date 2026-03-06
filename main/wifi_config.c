#include "wifi_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "WIFI_CONFIG";
static const char *NVS_NAMESPACE = "wifi_config";
static const char *NVS_SSID_KEY = "ssid";
static const char *NVS_PASS_KEY = "password";

bool wifi_config_save(const char *ssid, const char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    // Save SSID
    err = nvs_set_str(nvs_handle, NVS_SSID_KEY, ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving SSID: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    // Save password
    err = nvs_set_str(nvs_handle, NVS_PASS_KEY, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving password: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    // Commit changes
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials saved successfully");
        return true;
    } else {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
        return false;
    }
}

bool wifi_config_load(char *ssid, size_t ssid_len, char *password, size_t pass_len)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return false;
    }
    
    // Load SSID
    size_t required_size = ssid_len;
    err = nvs_get_str(nvs_handle, NVS_SSID_KEY, ssid, &required_size);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return false;
    }
    
    // Load password
    required_size = pass_len;
    err = nvs_get_str(nvs_handle, NVS_PASS_KEY, password, &required_size);
    nvs_close(nvs_handle);
    
    return (err == ESP_OK);
}

bool wifi_config_exists(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return false;
    }
    
    // Check if SSID exists
    size_t required_size;
    err = nvs_get_str(nvs_handle, NVS_SSID_KEY, NULL, &required_size);
    nvs_close(nvs_handle);
    
    return (err == ESP_OK);
}

void wifi_config_clear(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }
    
    nvs_erase_key(nvs_handle, NVS_SSID_KEY);
    nvs_erase_key(nvs_handle, NVS_PASS_KEY);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "WiFi credentials cleared");
}
