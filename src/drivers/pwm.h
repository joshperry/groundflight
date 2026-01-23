/**
 * PWM Servo Output Driver
 */

#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <stdbool.h>

bool pwm_init(uint16_t frequency_hz);
void pwm_set_pulse(uint8_t channel, uint16_t pulse_us);
void pwm_set_normalized(uint8_t channel, float value);  /* -1.0 to 1.0 */

#endif /* PWM_H */
