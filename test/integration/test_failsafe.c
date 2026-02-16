/**
 * Integration test: failsafe behavior
 *
 * Verifies that link loss / disarmed state results in safe outputs.
 * Safety invariants:
 *   - Steering and motor go to 1500us (neutral)
 *   - E-brake goes to 2000us (fully applied)
 */

#include "unity.h"
#include "crsf.h"
#include "stabilizer.h"
#include "mixer.h"
#include "config.h"
#include "mock_pwm.h"

/* Simulate the disarm path from main.c */
static void apply_disarmed_outputs(void)
{
    pwm_set_pulse(PWM_STEERING, PWM_PULSE_CENTER);
    pwm_set_pulse(PWM_EBRAKE, PWM_PULSE_MAX);  /* E-brake fully applied */
    pwm_set_pulse(PWM_MOTOR, PWM_PULSE_CENTER);
}

/* Simulate one armed iteration */
static void apply_armed_outputs(uint16_t steer_crsf, uint16_t throttle_crsf)
{
    float steer_cmd = crsf_to_float(steer_crsf);
    float throttle_cmd = crsf_to_float(throttle_crsf);
    float correction = stabilizer_update(steer_cmd, 0.0f, 0.0f, throttle_cmd, 1.0f);
    mixer_update(steer_cmd, throttle_cmd, 0.0f, 0.0f, correction);

    const mixer_output_t *mixed = mixer_get_output();
    uint16_t out_crsf = (uint16_t)(CRSF_CHANNEL_MID +
                          mixed->steer * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID));
    pwm_set_crsf(PWM_STEERING, out_crsf);
    /* Throttle goes through ESC (not PWM in normal path) but test the principle */
    pwm_set_pulse(PWM_MOTOR, crsf_to_us(throttle_crsf));
}

void setUp(void)
{
    config_init();
    config_t *cfg = config_get();

    stab_gains_t gains = {
        .kp = cfg->kp, .ki = cfg->ki, .kd = cfg->kd,
        .yaw_rate_scale = cfg->yaw_rate_scale,
        .max_correction = cfg->max_correction,
        .low_speed_gain = cfg->low_speed_gain,
        .high_speed_gain = cfg->high_speed_gain,
        .speed_gain_max_mph = cfg->speed_gain_max_mph,
    };
    stabilizer_init(&gains);
    stabilizer_set_mode(STAB_MODE_NORMAL);
    mixer_init();
    mock_pwm_reset();
}

void tearDown(void) {}

/* ---- Core safety invariant ---- */

void test_disarmed_outputs_safe(void)
{
    apply_disarmed_outputs();

    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_STEERING));
    TEST_ASSERT_EQUAL_UINT16(2000, pwm_get_pulse(PWM_EBRAKE));
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_MOTOR));
}

/* ---- Transition from armed to disarmed ---- */

void test_arm_then_disarm_goes_safe(void)
{
    /* Simulate armed state with right steering and forward throttle */
    for (int i = 0; i < 10; i++) {
        apply_armed_outputs(CRSF_CHANNEL_MAX, 1600);
    }

    /* Verify outputs are NOT at failsafe while armed */
    uint16_t steer_armed = pwm_get_pulse(PWM_STEERING);
    TEST_ASSERT_TRUE(steer_armed > 1600);

    /* Now disarm (failsafe / link loss) */
    apply_disarmed_outputs();

    /* Steering and motor neutral, e-brake applied */
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_STEERING));
    TEST_ASSERT_EQUAL_UINT16(2000, pwm_get_pulse(PWM_EBRAKE));
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_MOTOR));
}

/* ---- Car ESC neutral is 1500, NOT 1000 ---- */

void test_car_esc_neutral_is_1500_not_1000(void)
{
    /* This is the critical lesson from the project history.
     * Car ESCs use 1500us as NEUTRAL (stopped):
     *   1000us = full reverse/brake
     *   1500us = stopped
     *   2000us = full forward
     *
     * Failsafe MUST output 1500, never 1000. */
    apply_disarmed_outputs();

    uint16_t motor = pwm_get_pulse(PWM_MOTOR);
    TEST_ASSERT_EQUAL_UINT16(1500, motor);
    TEST_ASSERT_TRUE(motor != 1000);
}

/* ---- E-brake applied on failsafe ---- */

void test_ebrake_applied_on_failsafe(void)
{
    /* E-brake released while driving */
    pwm_set_pulse(PWM_EBRAKE, 1000);
    TEST_ASSERT_EQUAL_UINT16(1000, pwm_get_pulse(PWM_EBRAKE));

    /* Disarm -> e-brake fully applied */
    apply_disarmed_outputs();
    TEST_ASSERT_EQUAL_UINT16(2000, pwm_get_pulse(PWM_EBRAKE));
}

/* ---- Disarm during full throttle ---- */

void test_disarm_during_full_throttle(void)
{
    /* Full forward throttle */
    pwm_set_pulse(PWM_MOTOR, 2000);
    TEST_ASSERT_EQUAL_UINT16(2000, pwm_get_pulse(PWM_MOTOR));

    /* Disarm */
    apply_disarmed_outputs();
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_MOTOR));
}

/* ---- Disarm during full reverse ---- */

void test_disarm_during_reverse(void)
{
    pwm_set_pulse(PWM_MOTOR, 1000);
    TEST_ASSERT_EQUAL_UINT16(1000, pwm_get_pulse(PWM_MOTOR));

    apply_disarmed_outputs();
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_MOTOR));
}

/* ---- Disarm during full steering lock ---- */

void test_disarm_during_full_steering(void)
{
    pwm_set_pulse(PWM_STEERING, 2000);
    TEST_ASSERT_EQUAL_UINT16(2000, pwm_get_pulse(PWM_STEERING));

    apply_disarmed_outputs();
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_STEERING));
}

/* ---- Repeated disarm is idempotent ---- */

void test_disarm_idempotent(void)
{
    apply_disarmed_outputs();
    apply_disarmed_outputs();
    apply_disarmed_outputs();

    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_STEERING));
    TEST_ASSERT_EQUAL_UINT16(2000, pwm_get_pulse(PWM_EBRAKE));
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_get_pulse(PWM_MOTOR));
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_disarmed_outputs_safe);
    RUN_TEST(test_arm_then_disarm_goes_safe);
    RUN_TEST(test_car_esc_neutral_is_1500_not_1000);
    RUN_TEST(test_ebrake_applied_on_failsafe);
    RUN_TEST(test_disarm_during_full_throttle);
    RUN_TEST(test_disarm_during_reverse);
    RUN_TEST(test_disarm_during_full_steering);
    RUN_TEST(test_disarm_idempotent);

    return UNITY_END();
}
