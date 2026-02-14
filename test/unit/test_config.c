/**
 * Unit tests for runtime configuration
 */

#include "unity.h"
#include "config.h"
#include <string.h>

void setUp(void)
{
    config_init();
}

void tearDown(void) {}

/* ---- Default values match expected ---- */

void test_defaults_gyro(void)
{
    config_t *cfg = config_get();
    TEST_ASSERT_EQUAL_UINT8(50, cfg->gyro_lpf_hz);
}

void test_defaults_pid(void)
{
    config_t *cfg = config_get();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.003f, cfg->kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cfg->ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0001f, cfg->kd);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 400.0f, cfg->yaw_rate_scale);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, cfg->max_correction);
}

void test_defaults_speed_gain(void)
{
    config_t *cfg = config_get();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, cfg->low_speed_gain);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, cfg->high_speed_gain);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 60.0f, cfg->speed_gain_max_mph);
}

void test_defaults_steering(void)
{
    config_t *cfg = config_get();
    TEST_ASSERT_EQUAL_UINT16(1500, cfg->steer_center_us);
    TEST_ASSERT_EQUAL_UINT16(400, cfg->steer_range_us);
    TEST_ASSERT_EQUAL_INT8(1, cfg->steer_direction);
}

void test_defaults_esc(void)
{
    config_t *cfg = config_get();
    TEST_ASSERT_EQUAL_UINT8(1, cfg->esc_protocol);  /* SRXL2 */
    TEST_ASSERT_EQUAL_UINT16(1500, cfg->esc_center_us);
}

void test_defaults_vehicle(void)
{
    config_t *cfg = config_get();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.7f, cfg->gear_ratio);
    TEST_ASSERT_EQUAL_UINT16(107, cfg->tire_diameter_mm);
    TEST_ASSERT_EQUAL_UINT8(4, cfg->motor_poles);
}

void test_defaults_channels(void)
{
    config_t *cfg = config_get();
    TEST_ASSERT_EQUAL_UINT8(0, cfg->ch_steering);
    TEST_ASSERT_EQUAL_UINT8(2, cfg->ch_throttle);
    TEST_ASSERT_EQUAL_UINT8(5, cfg->ch_gain);
    TEST_ASSERT_EQUAL_UINT8(6, cfg->ch_mode);
    TEST_ASSERT_EQUAL_UINT8(3, cfg->ch_ebrake);
}

/* ---- Mutation and reset ---- */

void test_mutation_persists(void)
{
    config_t *cfg = config_get();
    cfg->kp = 0.01f;
    cfg->gyro_lpf_hz = 100;

    /* Read again - same pointer, should see changes */
    config_t *cfg2 = config_get();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, cfg2->kp);
    TEST_ASSERT_EQUAL_UINT8(100, cfg2->gyro_lpf_hz);
}

void test_reset_defaults_restores(void)
{
    config_t *cfg = config_get();
    cfg->kp = 0.01f;
    cfg->gyro_lpf_hz = 100;

    config_reset_defaults();

    cfg = config_get();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.003f, cfg->kp);
    TEST_ASSERT_EQUAL_UINT8(50, cfg->gyro_lpf_hz);
}

void test_load_resets_to_defaults(void)
{
    config_t *cfg = config_get();
    cfg->kp = 0.05f;

    config_load();  /* Currently just loads defaults */

    cfg = config_get();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.003f, cfg->kp);
}

/* ---- Struct size sanity ---- */

void test_config_struct_not_empty(void)
{
    TEST_ASSERT_TRUE(sizeof(config_t) > 0);
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_defaults_gyro);
    RUN_TEST(test_defaults_pid);
    RUN_TEST(test_defaults_speed_gain);
    RUN_TEST(test_defaults_steering);
    RUN_TEST(test_defaults_esc);
    RUN_TEST(test_defaults_vehicle);
    RUN_TEST(test_defaults_channels);
    RUN_TEST(test_mutation_persists);
    RUN_TEST(test_reset_defaults_restores);
    RUN_TEST(test_load_resets_to_defaults);
    RUN_TEST(test_config_struct_not_empty);

    return UNITY_END();
}
