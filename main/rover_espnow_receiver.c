#include "rover_espnow_receiver.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Use UART2 (RX2/TX2 pins) - UART1 is used by USB connection
#define UART_GNSS_PORT_NUM UART_NUM_2
#define UART_GNSS_TXD 17
#define UART_GNSS_RXD 16
#define UART_GNSS_BAUD_RATE 115200
#define UART_GNSS_BUF_SIZE 512

static const char *TAG = "ROVER_UART";

void rover_espnow_receiver_init(void) {
    // UART GNSS setup only (WiFi/MQTT handled in main.c)
    const uart_config_t uart_config = {
        .baud_rate = UART_GNSS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    esp_err_t err;
    
    err = uart_param_config(UART_GNSS_PORT_NUM, &uart_config);
    ESP_LOGI(TAG, "uart_param_config: %s", esp_err_to_name(err));
    
    err = uart_set_pin(UART_GNSS_PORT_NUM, UART_GNSS_TXD, UART_GNSS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "uart_set_pin(TX=%d, RX=%d): %s", UART_GNSS_TXD, UART_GNSS_RXD, esp_err_to_name(err));
    
    err = uart_driver_install(UART_GNSS_PORT_NUM, UART_GNSS_BUF_SIZE * 2, UART_GNSS_BUF_SIZE * 2, 0, NULL, 0);
    ESP_LOGI(TAG, "uart_driver_install: %s", esp_err_to_name(err));
    
    ESP_LOGI(TAG, "ROVER UART initialized");
}

void rover_forward_to_gnss(const uint8_t *data, size_t len) {
    // Send data to GNSS module via UART (non-blocking)
    uart_write_bytes(UART_GNSS_PORT_NUM, (const char *)data, len);
}

int rover_uart_read(uint8_t *data, size_t max_len) {
    return uart_read_bytes(UART_GNSS_PORT_NUM, data, max_len, pdMS_TO_TICKS(10));
}

// Parse NMEA GGA sentence to extract fix quality
// Fix quality: 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
int rover_parse_gga_fix(const uint8_t *data, size_t len) {
    // Validate input
    if (data == NULL || len < 50) {
        return -1;
    }
    
    // Look for $GNGGA or $GPGGA
    for (size_t i = 0; i <= len - 50; i++) {
        // Bounds check before accessing array
        if (i + 4 >= len) break;
        
        if (data[i] == '$' && data[i+2] == 'G' && data[i+3] == 'G' && data[i+4] == 'A') {
            // Find 6th comma (fix quality field)
            int comma_count = 0;
            size_t max_j = (i + 80 < len) ? i + 80 : len;
            for (size_t j = i; j < max_j; j++) {
                if (data[j] == ',') {
                    comma_count++;
                    if (comma_count == 6 && j + 1 < len) {
                        // Ensure the character is a valid ASCII digit
                        if (data[j + 1] >= '0' && data[j + 1] <= '9') {
                            return data[j + 1] - '0'; // Convert ASCII to int
                        }
                    }
                }
            }
        }
    }
    return -1; // Not found
}

// Extract a field from NMEA sentence between specified comma positions
static void extract_field(const uint8_t *data, size_t len, int field_num, char *output, size_t max_out) {
    if (!data || !output || max_out == 0 || len == 0) {
        if (output && max_out > 0) output[0] = '\0';
        return;
    }
    
    int comma_count = 0;
    size_t field_start = 0;
    output[0] = '\0';
    
    for (size_t i = 0; i < len && comma_count <= field_num; i++) {
        if (data[i] == ',') {
            if (comma_count == field_num && comma_count > 0 && field_start < i) {
                size_t field_len = i - field_start;
                if (field_len > max_out - 1) field_len = max_out - 1;
                if (field_len > 0 && field_start < len) {
                    memcpy(output, &data[field_start], field_len);
                    output[field_len] = '\0';
                }
                return;
            }
            comma_count++;
            field_start = i + 1;
        }
    }
    
    // Last field (after final comma)
    if (comma_count == field_num && field_start < len) {
        size_t field_len = len - field_start;
        // Stop at *, \r, \n
        for (size_t i = field_start; i < len; i++) {
            if (data[i] == '*' || data[i] == '\r' || data[i] == '\n') {
                field_len = i - field_start;
                break;
            }
        }
        if (field_len > max_out - 1) field_len = max_out - 1;
        if (field_len > 0) {
            memcpy(output, &data[field_start], field_len);
            output[field_len] = '\0';
        }
    }
}

// Parse and display GGA data (lat, lon, altitude, sats, quality)
void rover_display_gga(const uint8_t *data, size_t len) {
    if (!data || len < 60) return;
    
    // Find the LAST complete GGA sentence in buffer
    int last_gga = -1;
    for (size_t i = 0; i < len - 60; i++) {
        if (data[i] == '$' && data[i+1] == 'G' && 
            (data[i+2] == 'N' || data[i+2] == 'P') &&
            data[i+3] == 'G' && data[i+4] == 'G' && data[i+5] == 'A') {
            last_gga = i;
        }
    }
    
    if (last_gga < 0) return;
    
    // Extract just what we need from the last GGA
    const uint8_t *gga = data + last_gga;
    int field = 0;
    int pos = 0;
    char fields[15][20] = {0}; // Store up to 15 fields
    
    // Parse fields separated by commas
    for (size_t i = 0; i < 150 && gga[i] != '\n' && gga[i] != '\r' && field < 15; i++) {
        if (gga[i] == ',') {
            fields[field][pos] = '\0';
            field++;
            pos = 0;
        } else if (gga[i] != '$' && pos < 19) {
            fields[field][pos++] = gga[i];
        }
    }
    
    // Fields: 0=GNGGA, 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W, 6=quality, 7=sats, 9=alt
    if (strlen(fields[2]) > 0 && strlen(fields[4]) > 0) {
        // Convert lat/lon from DDMM.MMMM to DD.DDDDDD
        double lat = atof(fields[2]);
        int lat_d = (int)(lat / 100);
        double lat_m = lat - (lat_d * 100);
        lat = lat_d + (lat_m / 60.0);
        if (fields[3][0] == 'S') lat = -lat;
        
        double lon = atof(fields[4]);
        int lon_d = (int)(lon / 100);
        double lon_m = lon - (lon_d * 100);
        lon = lon_d + (lon_m / 60.0);
        if (fields[5][0] == 'W') lon = -lon;
        
        // RTK quality indicator (from NMEA GGA field 6)
        // 0=No fix, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
        const char *fix = "No Fix";
        int q = atoi(fields[6]);
        if (q == 4) fix = "RTK FIX";        // Best accuracy (cm-level)
        else if (q == 5) fix = "RTK Float"; // Good accuracy (dm-level)
        else if (q == 2) fix = "DGPS";      // Meter-level
        else if (q == 1) fix = "GPS";       // Meter-level
        
        printf("Lat: %.7f  Lon: %.7f  Alt: %sm  Sats: %s  [%s]\n", 
               lat, lon, fields[9], fields[7], fix);
    }
}

// Parse and display GSA data (HDOP, VDOP, PDOP)
void rover_display_gsa(const uint8_t *data, size_t len) {
    char hdop[10] = {0}, pdop[10] = {0};
    
    // Look for GSA sentence
    for (size_t i = 0; i < len - 30; i++) {
        if (i + 2 >= len) break;
        if (data[i] == '$' && (data[i+2] == 'S' || data[i+3] == 'S')) {
            // Find end of sentence
            size_t sent_end = i;
            while (sent_end < len && data[sent_end] != '\n') sent_end++;
            if (sent_end >= len) sent_end = len - 1;
            
            // Extract fields: 15=PDOP, 16=HDOP, 17=VDOP
            extract_field(data + i, sent_end - i, 15, pdop, sizeof(pdop));
            extract_field(data + i, sent_end - i, 16, hdop, sizeof(hdop));
            
            if (hdop[0] && pdop[0]) {
                printf("[DOP] HDOP: %s  PDOP: %s\n", hdop, pdop);
            }
            return;
        }
    }
}
