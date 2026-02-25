#ifndef ROVER_ESPNOW_RECEIVER_H
#define ROVER_ESPNOW_RECEIVER_H

#include <stdint.h>
#include <stddef.h>

// Initialize UART for ROVER (ZED-F9P communication)
void rover_espnow_receiver_init(void);
// Forward RTCM data to ZED-F9P via UART
void rover_forward_to_gnss(const uint8_t *data, size_t len);
// Read data from ZED-F9P UART
int rover_uart_read(uint8_t *data, size_t max_len);
// Parse NMEA GGA to get fix quality (0=none, 1=GPS, 4=RTK Fixed, 5=RTK Float)
int rover_parse_gga_fix(const uint8_t *data, size_t len);
// Display formatted GGA data (lat, lon, alt, sats, fix)
void rover_display_gga(const uint8_t *data, size_t len);
// Display formatted GSA data (HDOP, PDOP, VDOP)
void rover_display_gsa(const uint8_t *data, size_t len);

#endif // ROVER_ESPNOW_RECEIVER_H

