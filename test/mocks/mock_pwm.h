/**
 * Mock PWM driver for host-side testing
 *
 * Captures last-written pulse widths per channel.
 */

#ifndef MOCK_PWM_H
#define MOCK_PWM_H

#include <stdint.h>
#include <stdbool.h>

/* Mirror real enum */
typedef enum {
    PWM_STEERING = 0,
    PWM_THROTTLE = 1,
    PWM_EBRAKE   = 2,
    PWM_AUX      = 3,
    PWM_MOTOR    = 4,
    PWM_NUM_CHANNELS
} pwm_channel_t;

#define PWM_PULSE_MIN     1000
#define PWM_PULSE_CENTER  1500
#define PWM_PULSE_MAX     2000

/* Captured state for assertions */
extern uint16_t mock_pwm_pulse[PWM_NUM_CHANNELS];

void mock_pwm_reset(void);

/* Standard pwm.h API */
bool     pwm_init(void);
void     pwm_set_pulse(pwm_channel_t channel, uint16_t pulse_us);
void     pwm_set_normalized(pwm_channel_t channel, float value);
void     pwm_set_crsf(pwm_channel_t channel, uint16_t crsf_value);
uint16_t pwm_get_pulse(pwm_channel_t channel);
void     pwm_enable(pwm_channel_t channel, bool enabled);
void     pwm_motor_init(void);

#endif /* MOCK_PWM_H */
