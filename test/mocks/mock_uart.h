/**
 * Mock UART driver for host-side testing
 *
 * Captures sent bytes into a ring buffer for test assertions.
 */

#ifndef MOCK_UART_H
#define MOCK_UART_H

#include <stdint.h>
#include <stdbool.h>

/* Re-export the real uart_port_t enum */
typedef enum {
    UART_CRSF = 4,
    UART_ESC  = 1,
    UART_AUX  = 6,
} uart_port_t;

/* Capture buffer for test inspection */
#define MOCK_UART_BUF_SIZE 1024

typedef struct {
    uint8_t  buf[MOCK_UART_BUF_SIZE];
    uint16_t len;
} mock_uart_capture_t;

/* One capture per port index (0..6) */
extern mock_uart_capture_t mock_uart_tx[7];

/* Reset all capture buffers */
void mock_uart_reset(void);

/* Standard uart.h API (stubs) */
bool     uart_init(uart_port_t port, uint32_t baudrate);
void     uart_deinit(uart_port_t port);
uint16_t uart_available(uart_port_t port);
int16_t  uart_read_byte(uart_port_t port);
uint16_t uart_read(uart_port_t port, uint8_t *buf, uint16_t len);
void     uart_send(uart_port_t port, const uint8_t *data, uint16_t len);
void     uart_send_byte(uart_port_t port, uint8_t byte);
void     uart_debug_dump(uart_port_t port);

#endif /* MOCK_UART_H */
