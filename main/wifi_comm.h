#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <stdint.h>
#include <stddef.h>

void wifi_init_sta(const char *ssid, const char *password);
int wifi_is_connected(void);

#endif // WIFI_COMM_H
