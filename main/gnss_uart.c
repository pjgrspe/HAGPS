/*
 * gnss_uart.c - ZED-F9P UART Communication
 */

#include "gnss_uart.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "GNSS_UART";

void gnss_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = GNSS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_PORT_NUM,
                                 GNSS_UART_TX_PIN,
                                 GNSS_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_PORT_NUM,
                                        GNSS_UART_RX_BUF_SIZE,
                                        GNSS_UART_TX_BUF_SIZE,
                                        0, NULL, 0));

    ESP_LOGI(TAG, "ZED-F9P UART initialized (TX=%d, RX=%d, %d baud)",
             GNSS_UART_TX_PIN, GNSS_UART_RX_PIN, GNSS_UART_BAUD_RATE);
}

int gnss_uart_read(uint8_t *buf, size_t max_len)
{
    return uart_read_bytes(GNSS_UART_PORT_NUM, buf, max_len, pdMS_TO_TICKS(100));
}

int gnss_uart_write(const uint8_t *data, size_t len)
{
    return uart_write_bytes(GNSS_UART_PORT_NUM, (const char *)data, len);
}

void gnss_uart_flush(void)
{
    uart_flush_input(GNSS_UART_PORT_NUM);
    ESP_LOGI(TAG, "UART RX buffer flushed");
}
