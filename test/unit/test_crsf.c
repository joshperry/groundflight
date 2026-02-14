/**
 * Unit tests for CRSF conversion functions (inline in crsf.h)
 */

#include "unity.h"
#include "crsf.h"

void setUp(void) {}
void tearDown(void) {}

/* ---- crsf_to_float ---- */

void test_crsf_to_float_min(void)
{
    /* 172 should map to -1.0 */
    float val = crsf_to_float(CRSF_CHANNEL_MIN);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, val);
}

void test_crsf_to_float_mid(void)
{
    /* 992 should map to 0.0 */
    float val = crsf_to_float(CRSF_CHANNEL_MID);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, val);
}

void test_crsf_to_float_max(void)
{
    /* 1811 should map to 1.0 */
    float val = crsf_to_float(CRSF_CHANNEL_MAX);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, val);
}

void test_crsf_to_float_quarter(void)
{
    /* Halfway between mid and max: (992 + 1811) / 2 = 1401.5 -> ~0.5 */
    uint16_t quarter_up = (CRSF_CHANNEL_MID + CRSF_CHANNEL_MAX) / 2;
    float val = crsf_to_float(quarter_up);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, val);
}

void test_crsf_to_float_clamps_below(void)
{
    /* Value below min range should clamp to -1.0 */
    float val = crsf_to_float(0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, val);
}

void test_crsf_to_float_clamps_above(void)
{
    /* Value above max range should clamp to 1.0 */
    float val = crsf_to_float(2048);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, val);
}

/* ---- crsf_to_us ---- */

void test_crsf_to_us_min(void)
{
    /* 172 -> 988us */
    uint16_t us = crsf_to_us(CRSF_CHANNEL_MIN);
    TEST_ASSERT_EQUAL_UINT16(988, us);
}

void test_crsf_to_us_max(void)
{
    /* 1811 -> 2012us */
    uint16_t us = crsf_to_us(CRSF_CHANNEL_MAX);
    TEST_ASSERT_EQUAL_UINT16(2012, us);
}

void test_crsf_to_us_mid(void)
{
    /* 992 -> ~1500us (center) */
    uint16_t us = crsf_to_us(CRSF_CHANNEL_MID);
    /* Should be very close to 1500 */
    TEST_ASSERT_INT_WITHIN(2, 1500, us);
}

void test_crsf_to_us_monotonic(void)
{
    /* Verify output increases with input */
    uint16_t prev = crsf_to_us(CRSF_CHANNEL_MIN);
    for (uint16_t val = CRSF_CHANNEL_MIN + 10; val <= CRSF_CHANNEL_MAX; val += 10) {
        uint16_t cur = crsf_to_us(val);
        TEST_ASSERT_TRUE(cur >= prev);
        prev = cur;
    }
}

/* ---- Runner ---- */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_crsf_to_float_min);
    RUN_TEST(test_crsf_to_float_mid);
    RUN_TEST(test_crsf_to_float_max);
    RUN_TEST(test_crsf_to_float_quarter);
    RUN_TEST(test_crsf_to_float_clamps_below);
    RUN_TEST(test_crsf_to_float_clamps_above);
    RUN_TEST(test_crsf_to_us_min);
    RUN_TEST(test_crsf_to_us_max);
    RUN_TEST(test_crsf_to_us_mid);
    RUN_TEST(test_crsf_to_us_monotonic);

    return UNITY_END();
}
