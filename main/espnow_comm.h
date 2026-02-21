#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <stdint.h>
#include <stddef.h>

void espnow_comm_init(void);
void espnow_comm_send(const uint8_t *data, size_t len);

#endif // ESPNOW_COMM_H
