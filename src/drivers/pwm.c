/**
 * PWM Servo Output Driver
 * 
 * Timer configuration for STM32F722 @ 216MHz:
 *   - APB1 = 54MHz, timer clock = 108MHz (2x when prescaler > 1)
 *   - Prescaler = 107 → 1MHz counter (1µs resolution)
 *   - ARR = 19999 → 50Hz (20ms period)
 *   - CCR = pulse width in microseconds
 */

#include "pwm.h"
#include "target.h"
#include "stm32f7xx_hal.h"

/* Timer handles */
static TIM_HandleTypeDef htim3;  /* Steering, throttle, e-brake */
static TIM_HandleTypeDef htim2;  /* Aux servo */
static TIM_HandleTypeDef htim4;  /* Motor ESC */

/* Track current pulse widths */
static uint16_t pulse_values[PWM_NUM_CHANNELS];

/* Initialization state */
static bool pwm_initialized = false;

/* Timer clock = 108MHz, we want 1µs resolution */
#define PWM_PRESCALER       (108 - 1)
#define PWM_PERIOD          (20000 - 1)  /* 20ms = 50Hz */

/* ============================================================================
 * Timer Initialization
 * ============================================================================ */

/**
 * Initialize TIM3 for servo outputs
 * CH1 = PB4 (steering), CH2 = PB5 (throttle), CH3 = PB0 (e-brake)
 */
static bool tim3_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* Enable clocks */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure GPIO pins: PB0, PB4, PB5 as AF2 (TIM3) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure timer base */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = PWM_PRESCALER;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = PWM_PERIOD;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        return false;
    }
    
    /* Configure PWM channels */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_PULSE_CENTER;  /* Start at center */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    /* CH1 - Steering (PB4) */
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        return false;
    }
    
    /* CH2 - Throttle (PB5) */
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        return false;
    }
    
    /* CH3 - E-brake (PB0) */
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        return false;
    }
    
    /* Start PWM on all channels */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    
    return true;
}

/**
 * Initialize TIM2 for aux servo
 * CH2 = PB3
 */
static bool tim2_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* Enable clocks */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure GPIO pin: PB3 as AF1 (TIM2) */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure timer base */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = PWM_PRESCALER;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = PWM_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        return false;
    }
    
    /* Configure PWM channel */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_PULSE_CENTER;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    /* CH2 - Aux (PB3) */
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        return false;
    }
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    
    return true;
}

/**
 * Initialize TIM4 for motor ESC
 * CH1 = PB6
 */
static bool tim4_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* Enable clocks */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure GPIO pin: PB6 as AF2 (TIM4) */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure timer base */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = PWM_PRESCALER;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = PWM_PERIOD;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        return false;
    }
    
    /* Configure PWM channel */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_PULSE_CENTER;  /* Neutral */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    /* CH1 - Motor (PB6) */
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        return false;
    }
    
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    
    return true;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

bool pwm_init(void)
{
    /* Initialize all pulse values to safe defaults (center = neutral) */
    pulse_values[PWM_STEERING] = PWM_PULSE_CENTER;
    pulse_values[PWM_THROTTLE] = PWM_PULSE_CENTER;
    pulse_values[PWM_EBRAKE]   = PWM_PULSE_CENTER;
    pulse_values[PWM_AUX]      = PWM_PULSE_CENTER;
    pulse_values[PWM_MOTOR]    = PWM_PULSE_CENTER;  /* Neutral, not min */
    
    /* Initialize timers */
    if (!tim3_init()) return false;
    if (!tim2_init()) return false;
    if (!tim4_init()) return false;
    
    pwm_initialized = true;
    return true;
}

void pwm_set_pulse(pwm_channel_t channel, uint16_t pulse_us)
{
    if (!pwm_initialized || channel >= PWM_NUM_CHANNELS) return;
    
    /* Clamp to valid range */
    if (pulse_us < PWM_PULSE_MIN) pulse_us = PWM_PULSE_MIN;
    if (pulse_us > PWM_PULSE_MAX) pulse_us = PWM_PULSE_MAX;
    
    pulse_values[channel] = pulse_us;
    
    /* Update the appropriate timer channel */
    switch (channel) {
        case PWM_STEERING:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
            break;
        case PWM_THROTTLE:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);
            break;
        case PWM_EBRAKE:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_us);
            break;
        case PWM_AUX:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us);
            break;
        case PWM_MOTOR:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
            break;
        default:
            break;
    }
}

void pwm_set_normalized(pwm_channel_t channel, float value)
{
    /* Clamp to -1.0 to 1.0 */
    if (value < -1.0f) value = -1.0f;
    if (value >  1.0f) value =  1.0f;
    
    /* Map to pulse width: -1.0 → 1000µs, 0.0 → 1500µs, 1.0 → 2000µs */
    uint16_t pulse_us = (uint16_t)(PWM_PULSE_CENTER + value * 500.0f);
    pwm_set_pulse(channel, pulse_us);
}

void pwm_set_crsf(pwm_channel_t channel, uint16_t crsf_value)
{
    /* CRSF range: 172-1811, map to 1000-2000µs */
    /* Formula: pulse = 1000 + (crsf - 172) * 1000 / 1639 */
    
    /* Clamp CRSF value */
    if (crsf_value < 172)  crsf_value = 172;
    if (crsf_value > 1811) crsf_value = 1811;
    
    uint16_t pulse_us = 1000 + ((uint32_t)(crsf_value - 172) * 1000) / 1639;
    pwm_set_pulse(channel, pulse_us);
}

uint16_t pwm_get_pulse(pwm_channel_t channel)
{
    if (channel >= PWM_NUM_CHANNELS) return PWM_PULSE_CENTER;
    return pulse_values[channel];
}

void pwm_enable(pwm_channel_t channel, bool enabled)
{
    if (!pwm_initialized || channel >= PWM_NUM_CHANNELS) return;
    
    switch (channel) {
        case PWM_STEERING:
            if (enabled) {
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            } else {
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
            }
            break;
        case PWM_THROTTLE:
            if (enabled) {
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
            } else {
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
            }
            break;
        case PWM_EBRAKE:
            if (enabled) {
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
            } else {
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            }
            break;
        case PWM_AUX:
            if (enabled) {
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
            } else {
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            }
            break;
        case PWM_MOTOR:
            if (enabled) {
                HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
            } else {
                HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            }
            break;
        default:
            break;
    }
}

void pwm_motor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Reconfigure PB6 for TIM4 (may have been used for USART1) */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Set to neutral and start output */
    pulse_values[PWM_MOTOR] = PWM_PULSE_CENTER;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PULSE_CENTER);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}
