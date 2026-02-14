/**
 * Unit tests for input/output mixer
 */

#include "unity.h"
#include "mixer.h"

void setUp(void)
{
    mixer_init();
}

void tearDown(void) {}

/* ---- Passthrough (zero correction) ---- */

void test_passthrough_center(void)
{
    mixer_update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->steer);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->throttle);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->ebrake);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->aux);
}

void test_passthrough_full_right(void)
{
    mixer_update(1.0f, 0.5f, 0.3f, -0.2f, 0.0f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, out->steer);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, out->throttle);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, out->ebrake);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.2f, out->aux);
}

/* ---- Steering correction ---- */

void test_correction_adds_to_steering(void)
{
    mixer_update(0.3f, 0.0f, 0.0f, 0.0f, 0.1f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, out->steer);
}

void test_negative_correction(void)
{
    mixer_update(0.0f, 0.0f, 0.0f, 0.0f, -0.2f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.2f, out->steer);
}

void test_correction_clamps_high(void)
{
    /* Steering at 0.8 + correction 0.5 = 1.3, should clamp to 1.0 */
    mixer_update(0.8f, 0.0f, 0.0f, 0.0f, 0.5f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, out->steer);
}

void test_correction_clamps_low(void)
{
    /* Steering at -0.8 + correction -0.5 = -1.3, should clamp to -1.0 */
    mixer_update(-0.8f, 0.0f, 0.0f, 0.0f, -0.5f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, out->steer);
}

/* ---- Throttle passthrough ---- */

void test_throttle_passthrough(void)
{
    mixer_update(0.0f, 0.7f, 0.0f, 0.0f, 0.3f);
    const mixer_output_t *out = mixer_get_output();
    /* Throttle not affected by correction */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, out->throttle);
}

void test_throttle_clamps(void)
{
    mixer_update(0.0f, 1.5f, 0.0f, 0.0f, 0.0f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, out->throttle);

    mixer_update(0.0f, -1.5f, 0.0f, 0.0f, 0.0f);
    out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, out->throttle);
}

/* ---- E-brake passthrough ---- */

void test_ebrake_passthrough(void)
{
    mixer_update(0.0f, 0.0f, 0.6f, 0.0f, 0.0f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, out->ebrake);
}

void test_ebrake_clamps_0_to_1(void)
{
    /* E-brake is unipolar: 0.0 to 1.0 */
    mixer_update(0.0f, 0.0f, -0.5f, 0.0f, 0.0f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->ebrake);

    mixer_update(0.0f, 0.0f, 1.5f, 0.0f, 0.0f);
    out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, out->ebrake);
}

/* ---- Aux passthrough ---- */

void test_aux_passthrough(void)
{
    mixer_update(0.0f, 0.0f, 0.0f, -0.7f, 0.0f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.7f, out->aux);
}

void test_aux_clamps(void)
{
    mixer_update(0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, out->aux);
}

/* ---- Init state ---- */

void test_init_zeroes_all(void)
{
    /* Put some values in */
    mixer_update(0.5f, 0.5f, 0.5f, 0.5f, 0.1f);
    /* Re-init */
    mixer_init();
    const mixer_output_t *out = mixer_get_output();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->steer);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->throttle);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->ebrake);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out->aux);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_passthrough_center);
    RUN_TEST(test_passthrough_full_right);
    RUN_TEST(test_correction_adds_to_steering);
    RUN_TEST(test_negative_correction);
    RUN_TEST(test_correction_clamps_high);
    RUN_TEST(test_correction_clamps_low);
    RUN_TEST(test_throttle_passthrough);
    RUN_TEST(test_throttle_clamps);
    RUN_TEST(test_ebrake_passthrough);
    RUN_TEST(test_ebrake_clamps_0_to_1);
    RUN_TEST(test_aux_passthrough);
    RUN_TEST(test_aux_clamps);
    RUN_TEST(test_init_zeroes_all);

    return UNITY_END();
}
