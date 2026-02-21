#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include <stdint.h>
#include <stddef.h>

void wifi_comm_init(const char *ssid, const char *password, int is_base);
void wifi_comm_send(const uint8_t *data, size_t len);
int wifi_comm_receive(uint8_t *data, size_t max_len);

#endif // WIFI_COMM_H
