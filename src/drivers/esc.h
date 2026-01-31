/**
 * ESC Driver
 * 
 * Unified interface for ESC control and telemetry.
 * Supports PWM and SRXL2 (Spektrum Smart) modes.
 * 
 * Both modes use PB6 (ESC header on Nexus):
 *   PWM Mode:   TIM4_CH1 (AF2) - Standard 50Hz servo signal
 *   SRXL2 Mode: USART1_TX (AF7) - Half-duplex bidirectional
 * 
 * SRXL2 provides telemetry: RPM, voltage, current, temperature
 */

#ifndef ESC_H
#define ESC_H

#include <stdint.h>
#include <stdbool.h>

/* ESC output modes */
typedef enum {
    ESC_MODE_PWM,       /* Standard PWM servo signal */
    ESC_MODE_SRXL2,     /* Spektrum SRXL2 Smart ESC protocol */
} esc_mode_t;

/* Telemetry data from ESC */
typedef struct {
    uint32_t rpm;           /* Motor electrical RPM */
    float    voltage;       /* Pack voltage (V) */
    float    current;       /* Motor current (A) */
    float    temperature;   /* ESC temperature (°C) */
    uint32_t last_update_ms;
    bool     valid;         /* True if telemetry recently received */
} esc_telemetry_t;

/* Throttle limits (microseconds, same as servo PWM) */
#define ESC_THROTTLE_MIN     1000
#define ESC_THROTTLE_CENTER  1500
#define ESC_THROTTLE_MAX     2000

/**
 * Initialize ESC driver
 * 
 * @param mode ESC_MODE_PWM or ESC_MODE_SRXL2
 * @return true on success
 */
bool esc_init(esc_mode_t mode);

/**
 * Set ESC throttle
 * 
 * @param throttle_us Throttle in microseconds (1000-2000)
 *                    1000 = full reverse/brake
 *                    1500 = neutral
 *                    2000 = full forward
 */
void esc_set_throttle(uint16_t throttle_us);

/**
 * Set ESC throttle from CRSF value
 * 
 * @param crsf_value CRSF channel value (172-1811)
 */
void esc_set_throttle_crsf(uint16_t crsf_value);

/**
 * Process ESC communication
 * 
 * Call periodically from main loop. In SRXL2 mode, this handles
 * sending control frames and parsing telemetry responses.
 */
void esc_process(void);

/**
 * Get current ESC mode
 */
esc_mode_t esc_get_mode(void);

/**
 * Check if telemetry is available and valid
 */
bool esc_telemetry_valid(void);

/**
 * Get telemetry data
 * 
 * @return Pointer to telemetry struct (may be stale if !valid)
 */
const esc_telemetry_t* esc_get_telemetry(void);

/**
 * Check if ESC is connected (SRXL2 mode only)
 */
bool esc_is_connected(void);

/**
 * Get SRXL2 communication statistics
 */
void esc_get_srxl2_stats(uint32_t *tx, uint32_t *rx, uint32_t *crc_err, uint32_t *handshakes);

/**
 * Get SRXL2 packet type statistics for debugging
 */
void esc_get_srxl2_pkt_stats(uint32_t *hs, uint32_t *telem, uint32_t *ctrl, uint32_t *other, uint8_t *last_type);

/**
 * Get SRXL2 debug info (ESC device ID, baud capabilities, current baud, rehandshakes)
 */
void esc_get_srxl2_debug(uint8_t *esc_id, uint8_t *esc_baud_cap, uint32_t *baud, uint32_t *rehs);

/**
 * Convert RPM to speed (requires configuration)
 * 
 * @param rpm Motor electrical RPM
 * @param motor_poles Number of motor poles (typically 4 for car brushless)
 * @param gear_ratio Final drive ratio (motor:wheel)
 * @param tire_diameter Tire diameter in mm
 * @return Speed in mph
 */
float esc_rpm_to_mph(uint32_t rpm, uint8_t motor_poles, 
                     float gear_ratio, float tire_diameter_mm);

#endif /* ESC_H */
