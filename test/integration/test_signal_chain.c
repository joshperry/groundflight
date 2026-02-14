/**
 * Integration test: full signal chain
 *
 * CRSF input -> gyro filter -> stabilizer -> mixer -> PWM output
 *
 * Exercises the same data path as main.c's armed loop but on host,
 * using mock PWM to capture outputs.
 */

#include "unity.h"
#include "crsf.h"
#include "gyro.h"
#include "speed.h"
#include "stabilizer.h"
#include "mixer.h"
#include "config.h"
#include "mock_pwm.h"

void setUp(void)
{
    config_init();
    config_t *cfg = config_get();

    gyro_init(cfg->gyro_lpf_hz);
    speed_init(cfg->gear_ratio, cfg->tire_diameter_mm, cfg->motor_poles);

    stab_gains_t gains = {
        .kp = cfg->kp,
        .ki = cfg->ki,
        .kd = cfg->kd,
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

/* Simulate one control loop iteration (mirrors main.c armed path) */
static void run_control_loop(uint16_t crsf_steer, uint16_t crsf_throttle,
                              uint16_t crsf_ebrake, float gyro_z_raw,
                              uint32_t motor_rpm)
{
    /* Update gyro filter */
    gyro_update(0.0f, 0.0f, gyro_z_raw);

    /* Update speed */
    speed_update(motor_rpm);

    /* Get filtered gyro */
    const gyro_filtered_t *gyro = gyro_get_filtered();

    /* Convert CRSF to float */
    float steer_cmd = crsf_to_float(crsf_steer);
    float throttle_cmd = crsf_to_float(crsf_throttle);
    float ebrake_cmd = crsf_to_float(crsf_ebrake);
    ebrake_cmd = (ebrake_cmd + 1.0f) / 2.0f;  /* Convert to 0-1 */

    float speed_mph = speed_get_mph();

    /* Run stabilizer */
    float correction = stabilizer_update(steer_cmd, gyro->yaw_rate,
                                          speed_mph, throttle_cmd, 1.0f);

    /* Run mixer */
    mixer_update(steer_cmd, throttle_cmd, ebrake_cmd, 0.0f, correction);

    /* Get mixed outputs and write to mock PWM */
    const mixer_output_t *mixed = mixer_get_output();

    uint16_t steer_crsf = (uint16_t)(CRSF_CHANNEL_MID +
                            mixed->steer * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID));
    uint16_t ebrake_crsf = (uint16_t)(CRSF_CHANNEL_MIN +
                             mixed->ebrake * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN));

    pwm_set_crsf(PWM_STEERING, steer_crsf);
    pwm_set_crsf(PWM_EBRAKE, ebrake_crsf);
}

/* ---- Centered sticks, no yaw -> neutral outputs ---- */

void test_centered_no_yaw(void)
{
    /* Run several iterations to let filter settle */
    for (int i = 0; i < 200; i++) {
        run_control_loop(CRSF_CHANNEL_MID, CRSF_CHANNEL_MID,
                          CRSF_CHANNEL_MIN, 0.0f, 0);
    }

    /* Steering should be near center (1500us) */
    uint16_t steer = pwm_get_pulse(PWM_STEERING);
    TEST_ASSERT_INT_WITHIN(10, 1500, steer);
}

/* ---- Right stick, no gyro correction needed -> right output ---- */

void test_right_stick_matching_yaw(void)
{
    /* Full right stick (1811), gyro matches expected (400 dps) */
    /* Stabilizer should see zero error, no correction */
    for (int i = 0; i < 200; i++) {
        run_control_loop(CRSF_CHANNEL_MAX, CRSF_CHANNEL_MID,
                          CRSF_CHANNEL_MIN, 400.0f, 0);
    }

    /* Steering should be near full right (~2012us) */
    uint16_t steer = pwm_get_pulse(PWM_STEERING);
    TEST_ASSERT_TRUE(steer > 1900);
}

/* ---- Oversteer: gyro reports more yaw than expected ---- */

void test_oversteer_correction(void)
{
    /* Half right stick (mid between 992 and 1811 ≈ 1401) */
    uint16_t half_right = (CRSF_CHANNEL_MID + CRSF_CHANNEL_MAX) / 2;

    /* Expected yaw = 0.5 * 400 = 200 dps, but gyro reads 300 (oversteer) */
    /* Error = 200 - 300 = -100, correction is negative (reduce steering) */
    for (int i = 0; i < 200; i++) {
        run_control_loop(half_right, CRSF_CHANNEL_MID,
                          CRSF_CHANNEL_MIN, 300.0f, 0);
    }

    /* Without stabilizer, half-right would give ~1750us */
    /* With negative correction, should be less */
    uint16_t steer = pwm_get_pulse(PWM_STEERING);
    TEST_ASSERT_TRUE(steer < 1750);
    /* But still to the right of center */
    TEST_ASSERT_TRUE(steer > 1500);
}

/* ---- Understeer: gyro reports less yaw than expected ---- */

void test_understeer_correction(void)
{
    uint16_t half_right = (CRSF_CHANNEL_MID + CRSF_CHANNEL_MAX) / 2;

    /* Expected yaw = 200 dps, gyro reads only 50 (understeer) */
    /* Error = 200 - 50 = 150, correction is positive (add more steering) */
    for (int i = 0; i < 200; i++) {
        run_control_loop(half_right, CRSF_CHANNEL_MID,
                          CRSF_CHANNEL_MIN, 50.0f, 0);
    }

    /* Should be more than half-right due to positive correction */
    uint16_t steer = pwm_get_pulse(PWM_STEERING);
    uint16_t half_right_us = crsf_to_us(half_right);
    TEST_ASSERT_TRUE(steer > half_right_us);
}

/* ---- E-brake passes through ---- */

void test_ebrake_passthrough(void)
{
    /* Full e-brake (CRSF max) */
    for (int i = 0; i < 50; i++) {
        run_control_loop(CRSF_CHANNEL_MID, CRSF_CHANNEL_MID,
                          CRSF_CHANNEL_MAX, 0.0f, 0);
    }

    uint16_t ebrake = pwm_get_pulse(PWM_EBRAKE);
    /* Should be near max */
    TEST_ASSERT_TRUE(ebrake > 1900);
}

/* ---- Speed reduces correction at high speed ---- */

void test_speed_reduces_correction(void)
{
    /* Same oversteer scenario at 0 mph */
    for (int i = 0; i < 200; i++) {
        run_control_loop(CRSF_CHANNEL_MID, CRSF_CHANNEL_MID,
                          CRSF_CHANNEL_MIN, 100.0f, 0);
    }
    uint16_t steer_slow = pwm_get_pulse(PWM_STEERING);

    /* Reset stabilizer state */
    setUp();

    /* Same oversteer at 60mph (gain = 0.3) */
    for (int i = 0; i < 200; i++) {
        run_control_loop(CRSF_CHANNEL_MID, CRSF_CHANNEL_MID,
                          CRSF_CHANNEL_MIN, 100.0f, 30000);
    }
    uint16_t steer_fast = pwm_get_pulse(PWM_STEERING);

    /* At higher speed, correction should be smaller (closer to center) */
    int16_t dev_slow = (int16_t)steer_slow - 1500;
    int16_t dev_fast = (int16_t)steer_fast - 1500;

    /* Both should be negative (correcting against yaw) */
    /* Fast deviation should be smaller in magnitude */
    int abs_slow = dev_slow < 0 ? -dev_slow : dev_slow;
    int abs_fast = dev_fast < 0 ? -dev_fast : dev_fast;
    TEST_ASSERT_TRUE(abs_fast < abs_slow);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_centered_no_yaw);
    RUN_TEST(test_right_stick_matching_yaw);
    RUN_TEST(test_oversteer_correction);
    RUN_TEST(test_understeer_correction);
    RUN_TEST(test_ebrake_passthrough);
    RUN_TEST(test_speed_reduces_correction);

    return UNITY_END();
}
