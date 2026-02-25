#include "uart_gnss.h"
#include "driver/uart.h"

void uart_gnss_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_GNSS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_GNSS_PORT_NUM, &uart_config);
    uart_set_pin(UART_GNSS_PORT_NUM, UART_GNSS_TXD, UART_GNSS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_GNSS_PORT_NUM, UART_GNSS_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int uart_gnss_read(uint8_t *data, size_t max_len) {
    int len = uart_read_bytes(UART_GNSS_PORT_NUM, data, max_len, 10 / portTICK_PERIOD_MS);
    return len;
}
