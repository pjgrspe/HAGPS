#ifndef ROVER_ESPNOW_RECEIVER_H
#define ROVER_ESPNOW_RECEIVER_H

#include <stdint.h>
#include <stddef.h>

// Call this to initialize ESP-NOW for receiving RTCM data
void rover_espnow_receiver_init(void);
// Call this to receive RTCM data (returns number of bytes received)
int rover_espnow_receive(uint8_t *data, size_t max_len);
// Call this to forward received RTCM data to ZED-F9P via UART
void rover_forward_to_gnss(const uint8_t *data, size_t len);

#endif // ROVER_ESPNOW_RECEIVER_H
