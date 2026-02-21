#ifndef ROVER_WIFI_RECEIVER_H
#define ROVER_WIFI_RECEIVER_H

#include <stdint.h>
#include <stddef.h>

// Call this to initialize Wi-Fi and UDP socket for receiving RTCM data
void rover_wifi_receiver_init(const char *ssid, const char *password);
// Call this to receive RTCM data (returns number of bytes received)
int rover_wifi_receive(uint8_t *data, size_t max_len);
// Call this to forward received RTCM data to ZED-F9P via UART
void rover_forward_to_gnss(const uint8_t *data, size_t len);

#endif // ROVER_WIFI_RECEIVER_H
