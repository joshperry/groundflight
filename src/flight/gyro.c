/**
 * Gyro Signal Processing
 *
 * Filtering and calibration for IMU data
 */

#include "gyro.h"
#include <math.h>

/* Filtered gyro data */
static gyro_filtered_t g_filtered = {0};

/* Filter state (previous filtered values) */
static float prev_x = 0.0f;
static float prev_y = 0.0f;
static float prev_z = 0.0f;

/* Filter coefficient (alpha) - calculated from LPF frequency */
static float alpha = 0.5f;

/* Gyro bias (calibration offset) - in deg/s */
static float bias_x = 0.0f;
static float bias_y = 0.0f;
static float bias_z = 0.0f;

/**
 * Initialize gyro filtering
 *
 * @param lpf_hz Low-pass filter cutoff frequency in Hz
 */
void gyro_init(uint8_t lpf_hz)
{
    /* Calculate filter coefficient using simple exponential moving average
     * For a first-order IIR low-pass filter at sample rate fs with cutoff fc:
     *
     *   alpha = dt / (dt + RC)
     *   where RC = 1 / (2 * pi * fc)
     *   and dt = 1 / fs
     *
     * Assuming 1kHz sample rate (dt = 0.001s):
     *   RC = 1 / (2 * pi * lpf_hz)
     *   alpha = 0.001 / (0.001 + RC)
     *
     * Simplified: alpha ≈ 2 * pi * lpf_hz / 1000
     */
    const float dt = 0.001f;  /* 1kHz assumed update rate */
    const float pi = 3.14159265f;
    const float RC = 1.0f / (2.0f * pi * (float)lpf_hz);

    alpha = dt / (dt + RC);

    /* Clamp alpha to reasonable range */
    if (alpha < 0.01f) alpha = 0.01f;
    if (alpha > 1.0f)  alpha = 1.0f;

    /* Reset filter state */
    prev_x = prev_y = prev_z = 0.0f;
    g_filtered.roll_rate = 0.0f;
    g_filtered.pitch_rate = 0.0f;
    g_filtered.yaw_rate = 0.0f;
}

/**
 * Update gyro with new raw data
 *
 * @param raw_x Raw gyro X (roll rate) in deg/s
 * @param raw_y Raw gyro Y (pitch rate) in deg/s
 * @param raw_z Raw gyro Z (yaw rate) in deg/s
 */
void gyro_update(float raw_x, float raw_y, float raw_z)
{
    /* Apply calibration bias */
    float corrected_x = raw_x - bias_x;
    float corrected_y = raw_y - bias_y;
    float corrected_z = raw_z - bias_z;

    /* Apply exponential moving average filter
     * filtered = alpha * new + (1 - alpha) * old
     */
    prev_x = alpha * corrected_x + (1.0f - alpha) * prev_x;
    prev_y = alpha * corrected_y + (1.0f - alpha) * prev_y;
    prev_z = alpha * corrected_z + (1.0f - alpha) * prev_z;

    /* Update output structure */
    g_filtered.roll_rate = prev_x;
    g_filtered.pitch_rate = prev_y;
    g_filtered.yaw_rate = prev_z;
}

/**
 * Manual gyro calibration
 *
 * Note: For MVP, calibration is handled by icm42688_calibrate_gyro().
 * This function just resets the bias values since calibration happens
 * at the driver level.
 */
void gyro_calibrate(void)
{
    /* Driver-level calibration is preferred (icm42688_calibrate_gyro)
     * This function exists for future software-level calibration if needed
     */
    bias_x = 0.0f;
    bias_y = 0.0f;
    bias_z = 0.0f;
}

/**
 * Get filtered gyro data
 *
 * @return Pointer to filtered gyro data (read-only)
 */
const gyro_filtered_t* gyro_get_filtered(void)
{
    return &g_filtered;
}
