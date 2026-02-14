/**
 * Mock UART implementation
 */

#include "mock_uart.h"
#include <string.h>

mock_uart_capture_t mock_uart_tx[7] = {0};

void mock_uart_reset(void)
{
    memset(mock_uart_tx, 0, sizeof(mock_uart_tx));
}

bool uart_init(uart_port_t port, uint32_t baudrate)
{
    (void)port;
    (void)baudrate;
    return true;
}

void uart_deinit(uart_port_t port)
{
    (void)port;
}

uint16_t uart_available(uart_port_t port)
{
    (void)port;
    return 0;
}

int16_t uart_read_byte(uart_port_t port)
{
    (void)port;
    return -1;
}

uint16_t uart_read(uart_port_t port, uint8_t *buf, uint16_t len)
{
    (void)port;
    (void)buf;
    (void)len;
    return 0;
}

void uart_send(uart_port_t port, const uint8_t *data, uint16_t len)
{
    if (port > 6) return;
    mock_uart_capture_t *cap = &mock_uart_tx[port];
    for (uint16_t i = 0; i < len && cap->len < MOCK_UART_BUF_SIZE; i++) {
        cap->buf[cap->len++] = data[i];
    }
}

void uart_send_byte(uart_port_t port, uint8_t byte)
{
    uart_send(port, &byte, 1);
}

void uart_debug_dump(uart_port_t port)
{
    (void)port;
}
