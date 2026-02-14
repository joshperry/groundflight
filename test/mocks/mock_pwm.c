/**
 * Mock PWM implementation
 */

#include "mock_pwm.h"
#include <string.h>

uint16_t mock_pwm_pulse[PWM_NUM_CHANNELS] = {0};

void mock_pwm_reset(void)
{
    for (int i = 0; i < PWM_NUM_CHANNELS; i++) {
        mock_pwm_pulse[i] = PWM_PULSE_CENTER;
    }
}

bool pwm_init(void)
{
    mock_pwm_reset();
    return true;
}

void pwm_set_pulse(pwm_channel_t channel, uint16_t pulse_us)
{
    if (channel >= PWM_NUM_CHANNELS) return;
    if (pulse_us < PWM_PULSE_MIN) pulse_us = PWM_PULSE_MIN;
    if (pulse_us > PWM_PULSE_MAX) pulse_us = PWM_PULSE_MAX;
    mock_pwm_pulse[channel] = pulse_us;
}

void pwm_set_normalized(pwm_channel_t channel, float value)
{
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;
    uint16_t pulse = (uint16_t)(PWM_PULSE_CENTER + value * 500.0f);
    pwm_set_pulse(channel, pulse);
}

void pwm_set_crsf(pwm_channel_t channel, uint16_t crsf_value)
{
    /* Same formula as real pwm.c: linear map 172-1811 -> 988-2012 */
    uint16_t us = 988 + ((uint32_t)(crsf_value - 172) * 1024) / (1811 - 172);
    pwm_set_pulse(channel, us);
}

uint16_t pwm_get_pulse(pwm_channel_t channel)
{
    if (channel >= PWM_NUM_CHANNELS) return PWM_PULSE_CENTER;
    return mock_pwm_pulse[channel];
}

void pwm_enable(pwm_channel_t channel, bool enabled)
{
    (void)channel;
    (void)enabled;
}

void pwm_motor_init(void)
{
}
