/**
 * Unit tests for gyro signal processing (biquad LPF + calibration)
 */

#include "unity.h"
#include "gyro.h"
#include <math.h>

void setUp(void)
{
    gyro_init(50);  /* 50Hz cutoff, 1kHz sample rate */
}

void tearDown(void) {}

/* ---- Step response convergence ---- */

void test_step_input_converges(void)
{
    /* Feed constant 100 deg/s on Z axis for 200 samples (200ms at 1kHz) */
    for (int i = 0; i < 200; i++) {
        gyro_update(0.0f, 0.0f, 100.0f);
    }

    const gyro_filtered_t *f = gyro_get_filtered();
    /* After 200ms with 50Hz cutoff, output should be close to 100 */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 100.0f, f->yaw_rate);
}

void test_step_input_initial_lag(void)
{
    /* After just 5 samples, output should still be much less than input */
    for (int i = 0; i < 5; i++) {
        gyro_update(0.0f, 0.0f, 100.0f);
    }

    const gyro_filtered_t *f = gyro_get_filtered();
    TEST_ASSERT_TRUE(f->yaw_rate < 50.0f);
}

/* ---- Zero input stays zero ---- */

void test_zero_input_zero_output(void)
{
    for (int i = 0; i < 100; i++) {
        gyro_update(0.0f, 0.0f, 0.0f);
    }
    const gyro_filtered_t *f = gyro_get_filtered();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, f->yaw_rate);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, f->pitch_rate);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, f->roll_rate);
}

/* ---- High frequency rejection ---- */

void test_high_freq_attenuated(void)
{
    /* 400Hz signal (well above 50Hz cutoff) should be heavily attenuated */
    /* At 1kHz sample rate, 400Hz = period of 2.5 samples */
    float peak = 0.0f;

    /* Let filter settle for a bit, then measure peak */
    for (int i = 0; i < 1000; i++) {
        float signal = 100.0f * sinf(2.0f * 3.14159265f * 400.0f * (float)i / 1000.0f);
        gyro_update(0.0f, 0.0f, signal);
    }

    /* Sample last 100 points to find peak */
    for (int i = 1000; i < 1100; i++) {
        float signal = 100.0f * sinf(2.0f * 3.14159265f * 400.0f * (float)i / 1000.0f);
        gyro_update(0.0f, 0.0f, signal);
        const gyro_filtered_t *f = gyro_get_filtered();
        float abs_val = f->yaw_rate < 0 ? -f->yaw_rate : f->yaw_rate;
        if (abs_val > peak) peak = abs_val;
    }

    /* 2nd-order Butterworth at 400Hz should attenuate by >40 dB */
    /* 100 * 10^(-40/20) = 1.0, be generous */
    TEST_ASSERT_TRUE(peak < 5.0f);
}

/* ---- All three axes independent ---- */

void test_axes_independent(void)
{
    for (int i = 0; i < 200; i++) {
        gyro_update(50.0f, -30.0f, 100.0f);
    }

    const gyro_filtered_t *f = gyro_get_filtered();
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 50.0f, f->roll_rate);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, -30.0f, f->pitch_rate);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 100.0f, f->yaw_rate);
}

/* ---- Calibration zeros bias ---- */

void test_calibrate_resets_bias(void)
{
    /* gyro_calibrate() resets bias to 0 (driver does actual cal) */
    gyro_calibrate();
    /* Feed raw data - should pass through filter unbiased */
    for (int i = 0; i < 200; i++) {
        gyro_update(0.0f, 0.0f, 10.0f);
    }
    const gyro_filtered_t *f = gyro_get_filtered();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 10.0f, f->yaw_rate);
}

/* ---- Init resets filter state ---- */

void test_init_resets_state(void)
{
    /* Pump data through filter */
    for (int i = 0; i < 100; i++) {
        gyro_update(0.0f, 0.0f, 200.0f);
    }

    /* Reinitialize */
    gyro_init(50);

    /* First sample after init should be near zero (filter state cleared) */
    gyro_update(0.0f, 0.0f, 0.0f);
    const gyro_filtered_t *f = gyro_get_filtered();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, f->yaw_rate);
}

/* ---- Different cutoff frequencies ---- */

void test_low_cutoff_more_smoothing(void)
{
    /* 10Hz cutoff - much more smoothing */
    gyro_init(10);

    for (int i = 0; i < 50; i++) {
        gyro_update(0.0f, 0.0f, 100.0f);
    }
    const gyro_filtered_t *f_low = gyro_get_filtered();
    float val_10hz = f_low->yaw_rate;

    /* 100Hz cutoff - less smoothing */
    gyro_init(100);

    for (int i = 0; i < 50; i++) {
        gyro_update(0.0f, 0.0f, 100.0f);
    }
    const gyro_filtered_t *f_high = gyro_get_filtered();
    float val_100hz = f_high->yaw_rate;

    /* Higher cutoff should converge faster */
    TEST_ASSERT_TRUE(val_100hz > val_10hz);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_step_input_converges);
    RUN_TEST(test_step_input_initial_lag);
    RUN_TEST(test_zero_input_zero_output);
    RUN_TEST(test_high_freq_attenuated);
    RUN_TEST(test_axes_independent);
    RUN_TEST(test_calibrate_resets_bias);
    RUN_TEST(test_init_resets_state);
    RUN_TEST(test_low_cutoff_more_smoothing);

    return UNITY_END();
}
