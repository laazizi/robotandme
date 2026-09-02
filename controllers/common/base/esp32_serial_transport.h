#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <uxr/client/transport.h>

// Transport série custom micro-XRCE-DDS (pattern des exemples officiels
// micro_ros_espidf_component). transport->args = pointeur vers le n° d'UART.

bool esp32_serial_open(struct uxrCustomTransport *transport);
bool esp32_serial_close(struct uxrCustomTransport *transport);
size_t esp32_serial_write(struct uxrCustomTransport *transport,
                          const uint8_t *buf, size_t len, uint8_t *err);
size_t esp32_serial_read(struct uxrCustomTransport *transport,
                         uint8_t *buf, size_t len, int timeout, uint8_t *err);
