/**
 * SRXL2 Protocol Driver for GroundFlight
 * 
 * Bus master implementation for communicating with Spektrum Smart ESC.
 * Provides throttle control and telemetry readback.
 */

#ifndef SRXL2_H
#define SRXL2_H

#include <stdint.h>
#include <stdbool.h>

/* Telemetry data from ESC */
typedef struct {
    uint32_t rpm;           /* Motor electrical RPM */
    float    voltage;       /* Pack voltage (V) */
    float    current;       /* Motor current (A) */
    float    temperature;   /* ESC temperature (°C) */
    uint32_t last_update_ms;
    bool     valid;         /* True if recently received */
} srxl2_telemetry_t;

/* State information for debugging */
typedef struct {
    uint8_t  state;         /* Internal state machine state */
    bool     esc_found;     /* True if ESC responded to handshake */
    uint8_t  esc_device_id; /* Device ID of discovered ESC */
    uint8_t  esc_baud_supported; /* Baud rates ESC supports */
    uint32_t baud_rate;     /* Current baud rate */
    uint32_t rehandshake_count; /* Times we restarted handshake */
} srxl2_state_info_t;

/**
 * Initialize SRXL2 driver
 * 
 * Configures USART1 half-duplex on PB6 and starts handshake process.
 * 
 * @return true on success
 */
bool srxl2_init(void);

/**
 * Deinitialize SRXL2 driver
 * 
 * Releases UART and resets state. Call before switching to PWM mode.
 */
void srxl2_deinit(void);

/**
 * Process SRXL2 communication
 * 
 * Call periodically from main loop (~1kHz or faster).
 * Handles RX parsing, state machine, and periodic TX.
 */
void srxl2_process(void);

/**
 * Set throttle value
 * 
 * @param throttle_us Throttle in microseconds (1000-2000)
 */
void srxl2_set_throttle(uint16_t throttle_us);

/**
 * Set throttle from CRSF channel value
 * 
 * @param crsf_value CRSF channel value (172-1811)
 */
void srxl2_set_throttle_crsf(uint16_t crsf_value);

/**
 * Check if ESC is connected and communicating
 */
bool srxl2_is_connected(void);

/**
 * Check if telemetry data is valid
 */
bool srxl2_telemetry_valid(void);

/**
 * Get telemetry data
 * 
 * @return Pointer to telemetry struct (may be stale if !valid)
 */
const srxl2_telemetry_t* srxl2_get_telemetry(void);

/**
 * Get communication statistics
 */
void srxl2_get_stats(uint32_t *tx, uint32_t *rx, uint32_t *crc_err, uint32_t *handshakes);

/**
 * Get packet type statistics for debugging
 */
void srxl2_get_pkt_stats(uint32_t *hs, uint32_t *telem, uint32_t *ctrl, uint32_t *other, uint8_t *last_type);

/**
 * Get current state for debugging
 */
srxl2_state_info_t srxl2_get_state(void);

#endif /* SRXL2_H */
