#ifndef UART_GNSS_H
#define UART_GNSS_H

#include <stdint.h>
#include <stddef.h>

#define UART_GNSS_PORT_NUM UART_NUM_1
#define UART_GNSS_TXD 17
#define UART_GNSS_RXD 16
#define UART_GNSS_BAUD_RATE 115200
#define UART_GNSS_BUF_SIZE 2048

void uart_gnss_init(void);
int uart_gnss_read(uint8_t *data, size_t max_len);

#endif // UART_GNSS_H
