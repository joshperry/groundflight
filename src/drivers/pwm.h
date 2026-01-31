/**
 * PWM Servo Output Driver
 * 
 * Generates servo PWM signals on timer channels.
 * 
 * Channel mapping (from Rotorflight Nexus config):
 *   PWM_STEERING (0) - PB4 TIM3_CH1 - Steering servo
 *   PWM_THROTTLE (1) - PB5 TIM3_CH2 - Throttle/ESC
 *   PWM_EBRAKE   (2) - PB0 TIM3_CH3 - E-brake servo
 *   PWM_AUX      (3) - PB3 TIM2_CH2 - Aux servo
 *   PWM_MOTOR    (4) - PB6 TIM4_CH1 - Motor ESC (PWM mode)
 * 
 * Standard servo PWM: 50Hz, 1000-2000µs pulse width
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <stdbool.h>

/* Channel identifiers */
typedef enum {
    PWM_STEERING = 0,   /* PB4 TIM3_CH1 */
    PWM_THROTTLE = 1,   /* PB5 TIM3_CH2 */
    PWM_EBRAKE   = 2,   /* PB0 TIM3_CH3 */
    PWM_AUX      = 3,   /* PB3 TIM2_CH2 */
    PWM_MOTOR    = 4,   /* PB6 TIM4_CH1 */
    PWM_NUM_CHANNELS
} pwm_channel_t;

/* Servo pulse width limits (microseconds) */
#define PWM_PULSE_MIN     1000
#define PWM_PULSE_CENTER  1500
#define PWM_PULSE_MAX     2000

/**
 * Initialize PWM outputs
 * 
 * Sets up timers for servo PWM generation.
 * All channels start at center position (1500µs).
 * 
 * @return true on success
 */
bool pwm_init(void);

/**
 * Set pulse width for a channel
 * 
 * @param channel PWM channel
 * @param pulse_us Pulse width in microseconds (typically 1000-2000)
 */
void pwm_set_pulse(pwm_channel_t channel, uint16_t pulse_us);

/**
 * Set channel from normalized value
 * 
 * @param channel PWM channel
 * @param value Normalized value (-1.0 to 1.0), maps to 1000-2000µs
 */
void pwm_set_normalized(pwm_channel_t channel, float value);

/**
 * Set channel from CRSF value
 * 
 * @param channel PWM channel  
 * @param crsf_value CRSF channel value (172-1811)
 */
void pwm_set_crsf(pwm_channel_t channel, uint16_t crsf_value);

/**
 * Get current pulse width for a channel
 * 
 * @param channel PWM channel
 * @return Current pulse width in microseconds
 */
uint16_t pwm_get_pulse(pwm_channel_t channel);

/**
 * Enable/disable PWM output
 * 
 * When disabled, outputs go low (failsafe).
 * 
 * @param channel PWM channel
 * @param enabled true to enable output
 */
void pwm_enable(pwm_channel_t channel, bool enabled);

/**
 * Reinitialize motor PWM channel
 * 
 * Call this after switching from SRXL2 mode back to PWM mode.
 * Reconfigures PB6 GPIO for TIM4 output.
 */
void pwm_motor_init(void);

#endif /* PWM_H */
