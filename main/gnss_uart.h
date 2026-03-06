/*
 * gnss_uart.h - ZED-F9P UART Communication
 *
 * Provides a single UART interface shared by both BASE and ROVER modes
 * for communicating with the u-blox ZED-F9P GNSS receiver.
 *
 * Hardware: UART2 (GPIO17 TX, GPIO16 RX) at 115200 baud
 */

#ifndef GNSS_UART_H
#define GNSS_UART_H

#include <stdint.h>
#include <stddef.h>

/* Hardware configuration */
#define GNSS_UART_PORT_NUM    2       /* UART2 */
#define GNSS_UART_TX_PIN      17
#define GNSS_UART_RX_PIN      16
#define GNSS_UART_BAUD_RATE   115200
#define GNSS_UART_RX_BUF_SIZE 2048
#define GNSS_UART_TX_BUF_SIZE 1024

/**
 * Initialize UART for ZED-F9P communication.
 * Must be called once before any read/write operations.
 */
void gnss_uart_init(void);

/**
 * Read data from the ZED-F9P.
 *
 * @param buf       Destination buffer
 * @param max_len   Maximum bytes to read
 * @return          Number of bytes read (0 if none available)
 */
int gnss_uart_read(uint8_t *buf, size_t max_len);

/**
 * Write data to the ZED-F9P (e.g., RTCM corrections).
 *
 * @param data  Data to send
 * @param len   Number of bytes to send
 * @return      Number of bytes written
 */
int gnss_uart_write(const uint8_t *data, size_t len);

/**
 * Flush (discard) all data in the UART RX buffer.
 * Useful when restarting Survey-In to clear old RTCM data.
 */
void gnss_uart_flush(void);

#endif /* GNSS_UART_H */
