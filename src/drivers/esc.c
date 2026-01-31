/**
 * ESC Driver
 * 
 * Unified interface for ESC control and telemetry.
 * Both modes use PB6 (ESC header on Nexus):
 *   PWM Mode:   TIM4_CH1 (AF2) — Standard 50Hz servo signal
 *   SRXL2 Mode: USART1_TX (AF7) — Half-duplex bidirectional
 * 
 * SRXL2 provides telemetry: RPM, voltage, current, temperature
 */

#include "esc.h"
#include "srxl2.h"
#include "pwm.h"
#include "uart.h"
#include "target.h"
#include <string.h>

/* Current mode and state */
static esc_mode_t current_mode = ESC_MODE_PWM;
static bool initialized = false;
static uint16_t current_throttle = ESC_THROTTLE_CENTER;

/* Telemetry cache (for unified interface) */
static esc_telemetry_t telemetry;

/* ============================================================================
 * Public API
 * ============================================================================ */

bool esc_init(esc_mode_t mode)
{
    /* Clean up previous mode if switching */
    if (initialized) {
        if (current_mode == ESC_MODE_SRXL2) {
            srxl2_deinit();
        } else {
            pwm_enable(PWM_MOTOR, false);
        }
    }
    
    memset(&telemetry, 0, sizeof(telemetry));
    current_throttle = ESC_THROTTLE_CENTER;
    current_mode = mode;
    
    if (mode == ESC_MODE_SRXL2) {
        /* Initialize SRXL2 driver (handles UART setup) */
        if (!srxl2_init()) {
            return false;
        }
    } else {
        /* PWM mode - (re)configure PB6 for TIM4 */
        pwm_motor_init();
    }
    
    initialized = true;
    return true;
}

void esc_set_throttle(uint16_t throttle_us)
{
    if (!initialized) return;
    
    /* Clamp */
    if (throttle_us < ESC_THROTTLE_MIN) throttle_us = ESC_THROTTLE_MIN;
    if (throttle_us > ESC_THROTTLE_MAX) throttle_us = ESC_THROTTLE_MAX;
    
    current_throttle = throttle_us;
    
    if (current_mode == ESC_MODE_PWM) {
        pwm_set_pulse(PWM_MOTOR, throttle_us);
    } else {
        srxl2_set_throttle(throttle_us);
    }
}

void esc_set_throttle_crsf(uint16_t crsf_value)
{
    /* CRSF range: 172-1811, map to 1000-2000µs */
    if (crsf_value < 172)  crsf_value = 172;
    if (crsf_value > 1811) crsf_value = 1811;
    
    uint16_t throttle_us = 1000 + ((uint32_t)(crsf_value - 172) * 1000) / 1639;
    esc_set_throttle(throttle_us);
}

void esc_process(void)
{
    if (!initialized) return;
    
    if (current_mode == ESC_MODE_SRXL2) {
        srxl2_process();
        
        /* Copy telemetry to our unified struct */
        if (srxl2_telemetry_valid()) {
            const srxl2_telemetry_t *src = srxl2_get_telemetry();
            telemetry.rpm = src->rpm;
            telemetry.voltage = src->voltage;
            telemetry.current = src->current;
            telemetry.temperature = src->temperature;
            telemetry.last_update_ms = src->last_update_ms;
            telemetry.valid = true;
        } else {
            telemetry.valid = false;
        }
    }
    /* PWM mode doesn't need periodic processing */
}

esc_mode_t esc_get_mode(void)
{
    return current_mode;
}

bool esc_telemetry_valid(void)
{
    return telemetry.valid;
}

const esc_telemetry_t* esc_get_telemetry(void)
{
    return &telemetry;
}

bool esc_is_connected(void)
{
    if (current_mode == ESC_MODE_PWM) {
        return true;  /* PWM is always "connected" */
    }
    return srxl2_is_connected();
}

void esc_get_srxl2_stats(uint32_t *tx, uint32_t *rx, uint32_t *crc_err, uint32_t *handshakes)
{
    if (current_mode == ESC_MODE_SRXL2) {
        srxl2_get_stats(tx, rx, crc_err, handshakes);
    } else {
        if (tx) *tx = 0;
        if (rx) *rx = 0;
        if (crc_err) *crc_err = 0;
        if (handshakes) *handshakes = 0;
    }
}

void esc_get_srxl2_pkt_stats(uint32_t *hs, uint32_t *telem, uint32_t *ctrl, uint32_t *other, uint8_t *last_type)
{
    if (current_mode == ESC_MODE_SRXL2) {
        srxl2_get_pkt_stats(hs, telem, ctrl, other, last_type);
    } else {
        if (hs) *hs = 0;
        if (telem) *telem = 0;
        if (ctrl) *ctrl = 0;
        if (other) *other = 0;
        if (last_type) *last_type = 0;
    }
}

void esc_get_srxl2_debug(uint8_t *esc_id, uint8_t *esc_baud_cap, uint32_t *baud, uint32_t *rehs)
{
    if (current_mode == ESC_MODE_SRXL2) {
        srxl2_state_info_t state = srxl2_get_state();
        if (esc_id) *esc_id = state.esc_device_id;
        if (esc_baud_cap) *esc_baud_cap = state.esc_baud_supported;
        if (baud) *baud = state.baud_rate;
        if (rehs) *rehs = state.rehandshake_count;
    } else {
        if (esc_id) *esc_id = 0;
        if (esc_baud_cap) *esc_baud_cap = 0;
        if (baud) *baud = 0;
        if (rehs) *rehs = 0;
    }
}

float esc_rpm_to_mph(uint32_t rpm, uint8_t motor_poles, 
                     float gear_ratio, float tire_diameter_mm)
{
    if (motor_poles == 0 || gear_ratio == 0 || tire_diameter_mm == 0) {
        return 0;
    }
    
    /* Mechanical RPM = electrical RPM / (poles / 2) */
    float mech_rpm = (float)rpm / (motor_poles / 2.0f);
    
    /* Wheel RPM = motor RPM / gear ratio */
    float wheel_rpm = mech_rpm / gear_ratio;
    
    /* Tire circumference in mm */
    float circumference_mm = 3.14159f * tire_diameter_mm;
    
    /* Speed in mm/min = wheel_rpm * circumference */
    float mm_per_min = wheel_rpm * circumference_mm;
    
    /* Convert to mph: mm/min * 60 / 1609344 */
    float mph = mm_per_min * 60.0f / 1609344.0f;
    
    return mph;
}
