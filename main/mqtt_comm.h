#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#include <stdint.h>
#include <stddef.h>

void mqtt_comm_init(const char *broker_uri, const char *username, const char *password, const char *client_id, const char *topic);
void mqtt_comm_publish(const uint8_t *data, size_t len);
int mqtt_comm_receive(uint8_t *data, size_t max_len);
int mqtt_is_connected(void);

#endif // MQTT_COMM_H
