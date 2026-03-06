#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <stdbool.h>

/**
 * Initialize WiFi in station mode and attempt connection.
 * Returns true if connected within timeout_ms, false otherwise.
 * Use timeout_ms = 0 for infinite wait.
 */
bool wifi_init_sta(const char *ssid, const char *password, int timeout_ms);

/** Check if WiFi is currently connected. */
bool wifi_is_connected(void);

/** Stop WiFi (call before reconfiguring). */
void wifi_stop(void);

#endif // WIFI_COMM_H
