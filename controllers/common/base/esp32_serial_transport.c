#include "esp32_serial_transport.h"
#include "config.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_BUFFER_SIZE 4096

bool esp32_serial_open(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;

    uart_config_t uart_config = {
        .baud_rate  = SERIAL_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(*uart_port, &uart_config) != ESP_OK) {
        return false;
    }
    if (uart_set_pin(*uart_port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return false;
    }
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE, 0, 0, NULL, 0) != ESP_OK) {
        return false;
    }
    return true;
}

bool esp32_serial_close(struct uxrCustomTransport *transport)
{
    size_t *uart_port = (size_t *)transport->args;
    return uart_driver_delete(*uart_port) == ESP_OK;
}

size_t esp32_serial_write(struct uxrCustomTransport *transport,
                          const uint8_t *buf, size_t len, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    int written = uart_write_bytes(*uart_port, (const char *)buf, len);
    if (written < 0) {
        *err = 1;
        return 0;
    }
    return (size_t)written;
}

size_t esp32_serial_read(struct uxrCustomTransport *transport,
                         uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    size_t *uart_port = (size_t *)transport->args;
    int read = uart_read_bytes(*uart_port, buf, len, pdMS_TO_TICKS(timeout));
    if (read < 0) {
        *err = 1;
        return 0;
    }
    return (size_t)read;
}
