#ifndef MQTT_COMM_H
#define MQTT_COMM_H

#include <stdint.h>
#include <stddef.h>

void mqtt_comm_init(const char *broker_url, const char *client_id, const char *topic);
void mqtt_comm_publish(const uint8_t *data, size_t len);
int mqtt_comm_subscribe(uint8_t *data, size_t max_len);

#endif // MQTT_COMM_H
