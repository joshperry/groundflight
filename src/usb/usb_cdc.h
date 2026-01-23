/**
 * USB CDC Interface
 * 
 * Virtual COM port over USB for CLI access
 */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize USB CDC (call once at startup)
 */
void usb_cdc_init(void);

/**
 * Process USB events (call from main loop)
 */
void usb_cdc_process(void);

/**
 * Check if USB is connected and ready
 */
bool usb_cdc_is_connected(void);

/**
 * Send data over USB CDC
 */
void usb_cdc_send(const uint8_t *data, uint16_t len);

/**
 * Send a string over USB CDC
 */
void usb_cdc_print(const char *str);

/**
 * Check if data is available to read
 */
uint16_t usb_cdc_available(void);

/**
 * Read received data
 * Returns number of bytes actually read
 */
uint16_t usb_cdc_read(uint8_t *buf, uint16_t max_len);

/**
 * Read a single byte (-1 if none available)
 */
int16_t usb_cdc_read_byte(void);

#endif /* USB_CDC_H */
