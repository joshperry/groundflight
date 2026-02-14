/**
 * Unit tests for speed estimation (RPM -> mph/mps)
 */

#include "unity.h"
#include "speed.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

/* ---- Zero RPM ---- */

void test_zero_rpm(void)
{
    speed_init(8.0f, 100, 14);
    speed_update(0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed_get_mph());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed_get_mps());
}

/* ---- Known conversion ----
 *
 * gear_ratio = 8.0, tire_diameter = 100mm, motor_poles = 14 (7 pole pairs)
 * tire_circumference = 0.1 * pi = 0.31416 m
 *
 * motor_rpm = 7000 electrical
 * mechanical_rpm = 7000 / 7 = 1000
 * wheel_rpm = 1000 / 8.0 = 125
 * mps = (125 / 60) * 0.31416 = 0.65449 m/s
 * mph = 0.65449 * 2.237 = 1.46370
 */

void test_known_conversion(void)
{
    speed_init(8.0f, 100, 14);
    speed_update(7000);

    float expected_mps = (7000.0f / 7.0f / 8.0f / 60.0f) * (0.1f * 3.14159265f);
    float expected_mph = expected_mps * 2.237f;

    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_mps, speed_get_mps());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_mph, speed_get_mph());
}

/* ---- Arrma Infraction defaults ----
 *
 * gear_ratio = 7.7, tire_diameter = 107mm, motor_poles = 4 (2 pole pairs)
 */

void test_infraction_defaults(void)
{
    speed_init(7.7f, 107, 4);
    speed_update(10000);

    /* mechanical = 10000 / 2 = 5000 */
    /* wheel = 5000 / 7.7 = 649.35 */
    /* circ = 0.107 * pi = 0.33615 m */
    /* mps = (649.35 / 60) * 0.33615 = 3.6385 m/s */
    /* mph = 3.6385 * 2.237 = 8.1394 */
    float circ = 0.107f * 3.14159265f;
    float expected_mps = (10000.0f / 2.0f / 7.7f / 60.0f) * circ;
    float expected_mph = expected_mps * 2.237f;

    TEST_ASSERT_FLOAT_WITHIN(0.05f, expected_mps, speed_get_mps());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_mph, speed_get_mph());
}

/* ---- Consistency check: mph = mps * 2.237 ---- */

void test_mps_mph_consistent(void)
{
    speed_init(7.7f, 107, 4);
    speed_update(15000);

    float mps = speed_get_mps();
    float mph = speed_get_mph();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, mps * 2.237f, mph);
}

/* ---- Edge: single pole motor (motor_poles = 1 -> pole_pairs = 1) ---- */

void test_single_pole_clamp(void)
{
    /* motor_poles = 1, pole_pairs = max(1, 1/2) = 1 (clamped) */
    speed_init(1.0f, 100, 1);
    speed_update(6000);

    /* mechanical = 6000 / 1 = 6000, wheel = 6000, mps = 100*pi/60 = 31.416 */
    float circ = 0.1f * 3.14159265f;
    float expected_mps = (6000.0f / 1.0f / 60.0f) * circ;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_mps, speed_get_mps());
}

/* ---- RPM update replaces previous value ---- */

void test_update_replaces(void)
{
    speed_init(8.0f, 100, 14);
    speed_update(7000);
    float first = speed_get_mph();
    TEST_ASSERT_TRUE(first > 0.0f);

    speed_update(0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed_get_mph());
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_zero_rpm);
    RUN_TEST(test_known_conversion);
    RUN_TEST(test_infraction_defaults);
    RUN_TEST(test_mps_mph_consistent);
    RUN_TEST(test_single_pole_clamp);
    RUN_TEST(test_update_replaces);

    return UNITY_END();
}
