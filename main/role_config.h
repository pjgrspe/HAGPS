#ifndef ROLE_CONFIG_H
#define ROLE_CONFIG_H

#include <stdbool.h>

// Device Role Configuration
#define DEVICE_ROLE_BASE  1
#define DEVICE_ROLE_ROVER 2

// Save device role to NVS
bool role_config_save(int role);

// Load device role from NVS
int role_config_load(void);

// Check if role is configured
bool role_config_exists(void);

// Clear role configuration
void role_config_clear(void);

// Mark that ZED-F9P has been configured (for BASE mode)
void role_config_mark_zed_configured(void);

// Check if ZED-F9P has been configured before
bool role_config_is_zed_configured(void);

#endif // ROLE_CONFIG_H
