/**
 * UART Driver
 * 
 * Interrupt-driven UART with ring buffers for RX
 * 
 * Supported peripherals:
 *   USART1 - ESC telemetry, half-duplex on PB6 (AF7) - SRXL2
 *   UART4  - CRSF receiver (PA0=RX, PA1=TX, AF8)
 *   UART6  - Spare (PC6=RX, PC7=TX, AF8)
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

/* UART identifiers */
typedef enum {
    UART_CRSF = 4,    /* UART4 - CRSF/ELRS receiver */
    UART_ESC  = 1,    /* USART1 - ESC telemetry (half-duplex on PB6) */
    UART_AUX  = 6,    /* UART6 - Spare */
} uart_port_t;

/**
 * Initialize a UART peripheral
 * 
 * @param port UART port identifier
 * @param baudrate Baud rate (e.g., 420000 for CRSF, 115200 for SRXL2)
 * @return true on success
 */
bool uart_init(uart_port_t port, uint32_t baudrate);

/**
 * Deinitialize a UART peripheral
 * 
 * Releases the GPIO pins for other use (e.g., switching PB6 back to PWM)
 * 
 * @param port UART port identifier
 */
void uart_deinit(uart_port_t port);

/**
 * Check if data is available in RX buffer
 * 
 * @param port UART port
 * @return Number of bytes available
 */
uint16_t uart_available(uart_port_t port);

/**
 * Read a single byte from RX buffer
 * 
 * @param port UART port
 * @return Byte read, or -1 if buffer empty
 */
int16_t uart_read_byte(uart_port_t port);

/**
 * Read multiple bytes from RX buffer
 * 
 * @param port UART port
 * @param buf Destination buffer
 * @param len Maximum bytes to read
 * @return Number of bytes actually read
 */
uint16_t uart_read(uart_port_t port, uint8_t *buf, uint16_t len);

/**
 * Send data (blocking)
 * 
 * For half-duplex ports (UART_ESC), automatically handles direction switching.
 * 
 * @param port UART port
 * @param data Data to send
 * @param len Number of bytes
 */
void uart_send(uart_port_t port, const uint8_t *data, uint16_t len);

/**
 * Send a single byte (blocking)
 */
void uart_send_byte(uart_port_t port, uint8_t byte);

void uart_debug_dump(uart_port_t port);

#endif /* UART_H */
