/**
 * Unit tests for yaw stabilizer (PD controller + gain scheduling)
 */

#include "unity.h"
#include "stabilizer.h"

/* Default test gains matching config defaults */
static const stab_gains_t DEFAULT_GAINS = {
    .kp = 0.003f,
    .ki = 0.0f,
    .kd = 0.0001f,
    .yaw_rate_scale = 400.0f,
    .max_correction = 0.5f,
    .low_speed_gain = 1.0f,
    .high_speed_gain = 0.3f,
    .speed_gain_max_mph = 60.0f,
};

static void reset_stabilizer(const stab_gains_t *gains)
{
    stabilizer_init(gains);
    /* stabilizer_init does NOT reset g_prev_error.
     * Toggle through OFF to clear all derivative state. */
    stabilizer_set_mode(STAB_MODE_OFF);
    stabilizer_set_mode(STAB_MODE_NORMAL);
}

void setUp(void)
{
    reset_stabilizer(&DEFAULT_GAINS);
}

void tearDown(void) {}

/* ---- Mode behavior ---- */

void test_off_mode_returns_zero(void)
{
    stabilizer_set_mode(STAB_MODE_OFF);
    float c = stabilizer_update(0.5f, 100.0f, 30.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c);
}

void test_off_mode_resets_d_term(void)
{
    /* Build up derivative state */
    stabilizer_update(0.5f, 0.0f, 0.0f, 0.0f, 1.0f);
    stabilizer_update(0.5f, 50.0f, 0.0f, 0.0f, 1.0f);

    /* Switch OFF then back ON - D term should start fresh */
    stabilizer_set_mode(STAB_MODE_OFF);
    stabilizer_set_mode(STAB_MODE_NORMAL);

    /* First call after re-enable: prev_error was reset to 0 */
    float c1 = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);

    /* Second identical call: d_error should be 0 */
    float c2 = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);

    /* c1 includes D kick (error went from 0 to -10), c2 should not */
    TEST_ASSERT_TRUE(c1 != c2);
    /* c2 has d_error=0, so purely P: kp * error = 0.003 * (-10) = -0.03 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.03f, c2);
}

/* ---- PD output with known inputs ---- */

void test_pd_output_basic(void)
{
    /* Centered stick, 10 deg/s yaw rate, no speed, full gain knob
     * expected_yaw = 0 * 400 = 0, error = 0 - 10 = -10
     * P = 0.003 * (-10) = -0.03
     * D = 0.0001 * (-10 - 0) = -0.001  (prev_error=0 after reset)
     * correction = -0.031 */
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.031f, c);
}

void test_pd_output_p_only(void)
{
    /* Call twice with same inputs. Second call has d_error=0 (steady state).
     * This isolates the P term. */
    stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    /* P = 0.003 * (-10) = -0.03, D = 0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.03f, c);
}

void test_pd_output_with_steering(void)
{
    /* Half right stick: expected = 0.5 * 400 = 200 dps
     * Gyro reads 150 dps: error = 200 - 150 = 50
     * P = 0.003 * 50 = 0.15
     * D = 0.0001 * (50 - 0) = 0.005
     * Total = 0.155 */
    float c = stabilizer_update(0.5f, 150.0f, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.155f, c);
}

void test_derivative_opposes_change(void)
{
    /* Step 1: error = -10 */
    stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    /* Step 2: error = -20 (worsening) */
    float c2 = stabilizer_update(0.0f, 20.0f, 0.0f, 0.0f, 1.0f);

    /* d_error = -20 - (-10) = -10, so D adds negative
     * P2 = 0.003 * (-20) = -0.06
     * D2 = 0.0001 * (-10) = -0.001 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.061f, c2);
}

/* ---- Speed-based gain scheduling ----
 *
 * To isolate speed gain from D-term effects, we call twice
 * with the same inputs so d_error=0 on the second call.
 * That gives us pure P * speed_gain.
 */

void test_speed_gain_zero_mph(void)
{
    /* At 0 mph: gain = 1.0 */
    stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    /* P = -0.03, D = 0, speed_gain = 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.03f, c);
}

void test_speed_gain_30_mph(void)
{
    /* At 30 mph: t = 0.5, gain = 1.0 + 0.5*(0.3-1.0) = 0.65 */
    stabilizer_update(0.0f, 10.0f, 30.0f, 0.0f, 1.0f);
    float c = stabilizer_update(0.0f, 10.0f, 30.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.03f * 0.65f, c);
}

void test_speed_gain_60_mph(void)
{
    /* At 60 mph: t = 1.0, gain = 0.3 */
    stabilizer_update(0.0f, 10.0f, 60.0f, 0.0f, 1.0f);
    float c = stabilizer_update(0.0f, 10.0f, 60.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.03f * 0.3f, c);
}

void test_speed_gain_above_max(void)
{
    /* Above 60 mph: clamped to 1.0, gain = 0.3 */
    stabilizer_update(0.0f, 10.0f, 100.0f, 0.0f, 1.0f);
    float c = stabilizer_update(0.0f, 10.0f, 100.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.03f * 0.3f, c);
}

/* ---- Gain knob ---- */

void test_gain_knob_full(void)
{
    stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.03f, c);
}

void test_gain_knob_half(void)
{
    stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 0.5f);
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.03f * 0.5f, c);
}

void test_gain_knob_zero(void)
{
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c);
}

void test_gain_knob_clamped_above_one(void)
{
    /* gain_knob = 2.0 should clamp to 1.0, same result as 1.0 */
    stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    float c_full = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);

    reset_stabilizer(&DEFAULT_GAINS);

    stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    float c_over = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, c_full, c_over);
}

void test_gain_knob_clamped_below_zero(void)
{
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, -0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c);
}

/* ---- Max correction clamping ---- */

void test_max_correction_positive(void)
{
    /* Large error to saturate: steer = 1.0, gyro = -400
     * expected = 400, error = 800
     * P = 0.003 * 800 = 2.4 >> max_correction = 0.5 */
    float c = stabilizer_update(1.0f, -400.0f, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, c);
}

void test_max_correction_negative(void)
{
    float c = stabilizer_update(-1.0f, 400.0f, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, c);
}

/* ---- Edge cases ---- */

void test_zero_error(void)
{
    /* Stick at 0.5, gyro at exactly expected rate
     * expected = 0.5 * 400 = 200, gyro = 200, error = 0
     * P = 0, D = 0.0001 * (0 - 0) = 0 */
    float c = stabilizer_update(0.5f, 200.0f, 0.0f, 0.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, c);
}

void test_null_gains_init(void)
{
    /* NULL gains should not crash; init keeps previous gains */
    stabilizer_init(NULL);
    stabilizer_set_mode(STAB_MODE_OFF);
    stabilizer_set_mode(STAB_MODE_NORMAL);

    /* Gains are still DEFAULT_GAINS from setUp, so output should be nonzero */
    float c = stabilizer_update(0.0f, 10.0f, 0.0f, 0.0f, 1.0f);
    /* P = 0.003 * (-10) = -0.03, D = 0.0001 * (-10) = -0.001 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.031f, c);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    /* Mode behavior */
    RUN_TEST(test_off_mode_returns_zero);
    RUN_TEST(test_off_mode_resets_d_term);

    /* PD output */
    RUN_TEST(test_pd_output_basic);
    RUN_TEST(test_pd_output_p_only);
    RUN_TEST(test_pd_output_with_steering);
    RUN_TEST(test_derivative_opposes_change);

    /* Speed gain scheduling */
    RUN_TEST(test_speed_gain_zero_mph);
    RUN_TEST(test_speed_gain_30_mph);
    RUN_TEST(test_speed_gain_60_mph);
    RUN_TEST(test_speed_gain_above_max);

    /* Gain knob */
    RUN_TEST(test_gain_knob_full);
    RUN_TEST(test_gain_knob_half);
    RUN_TEST(test_gain_knob_zero);
    RUN_TEST(test_gain_knob_clamped_above_one);
    RUN_TEST(test_gain_knob_clamped_below_zero);

    /* Max correction */
    RUN_TEST(test_max_correction_positive);
    RUN_TEST(test_max_correction_negative);

    /* Edge cases */
    RUN_TEST(test_zero_error);
    RUN_TEST(test_null_gains_init);

    return UNITY_END();
}
