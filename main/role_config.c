#include "role_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "ROLE_CONFIG";
static const char *NVS_NAMESPACE = "role_config";
static const char *NVS_ROLE_KEY = "device_role";
static const char *NVS_ZED_CFG_KEY = "zed_configured";

bool role_config_save(int role)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_set_i32(nvs_handle, NVS_ROLE_KEY, role);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving role: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        const char *role_name = (role == DEVICE_ROLE_BASE) ? "BASE STATION" : "ROVER";
        ESP_LOGI(TAG, "Device role saved: %s", role_name);
        return true;
    } else {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
        return false;
    }
}

int role_config_load(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    int32_t role = DEVICE_ROLE_ROVER; // Default to ROVER
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return role;
    }
    
    err = nvs_get_i32(nvs_handle, NVS_ROLE_KEY, &role);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        const char *role_name = (role == DEVICE_ROLE_BASE) ? "BASE STATION" : "ROVER";
        ESP_LOGI(TAG, "Device role loaded: %s", role_name);
    }
    
    return (int)role;
}

bool role_config_exists(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return false;
    }
    
    int32_t role;
    err = nvs_get_i32(nvs_handle, NVS_ROLE_KEY, &role);
    nvs_close(nvs_handle);
    
    return (err == ESP_OK);
}

void role_config_clear(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }
    
    nvs_erase_key(nvs_handle, NVS_ROLE_KEY);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Device role cleared");
}

void role_config_mark_zed_configured(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }
    
    nvs_set_u8(nvs_handle, NVS_ZED_CFG_KEY, 1);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "ZED-F9P marked as configured");
}

bool role_config_is_zed_configured(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    uint8_t configured = 0;
    
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return false;
    }
    
    err = nvs_get_u8(nvs_handle, NVS_ZED_CFG_KEY, &configured);
    nvs_close(nvs_handle);
    
    return (err == ESP_OK && configured == 1);
}
