#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <stdbool.h>
#include <stddef.h>

// Save WiFi credentials to NVS
bool wifi_config_save(const char *ssid, const char *password);

// Load WiFi credentials from NVS
bool wifi_config_load(char *ssid, size_t ssid_len, char *password, size_t pass_len);

// Check if WiFi credentials exist
bool wifi_config_exists(void);

// Clear WiFi credentials
void wifi_config_clear(void);

#endif // WIFI_CONFIG_H
