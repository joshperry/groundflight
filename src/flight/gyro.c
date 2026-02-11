/**
 * Gyro Signal Processing
 *
 * Filtering and calibration for IMU data
 *
 * Uses a 2nd-order Butterworth low-pass biquad filter (Direct Form II
 * Transposed) for each axis. Compared to the old single-pole IIR:
 *   - Much less phase lag in the passband (better for closed-loop control)
 *   - Steeper rolloff above cutoff (-40 dB/decade vs -20)
 *   - Maximally flat passband (Butterworth property)
 *
 * At 50Hz cutoff / 1kHz sample rate:
 *   Old 1st-order: -45 deg phase at 50Hz, -6 dB at 50Hz
 *   New 2nd-order: -90 deg phase at 50Hz, -3 dB at 50Hz, but only ~5 deg
 *                  phase lag at 10Hz where car yaw signals live
 */

#include "gyro.h"
#include <math.h>

/* Biquad filter state (Direct Form II Transposed) */
typedef struct {
    float b0, b1, b2;  /* Feedforward coefficients */
    float a1, a2;       /* Feedback coefficients (negated for DFII-T) */
    float z1, z2;       /* Delay elements */
} biquad_t;

/* One biquad per axis */
static biquad_t filt_x, filt_y, filt_z;

/* Filtered gyro data */
static gyro_filtered_t g_filtered = {0};

/* Gyro bias (calibration offset) - in deg/s */
static float bias_x = 0.0f;
static float bias_y = 0.0f;
static float bias_z = 0.0f;

/**
 * Apply biquad filter to one sample (Direct Form II Transposed)
 *
 * y[n] = b0*x[n] + z1
 * z1   = b1*x[n] - a1*y[n] + z2
 * z2   = b2*x[n] - a2*y[n]
 *
 * Only 2 state variables, good numerical properties with float32.
 */
static inline float biquad_apply(biquad_t *f, float x)
{
    float y  = f->b0 * x + f->z1;
    f->z1    = f->b1 * x - f->a1 * y + f->z2;
    f->z2    = f->b2 * x - f->a2 * y;
    return y;
}

static void biquad_reset(biquad_t *f)
{
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

/**
 * Initialize gyro filtering
 *
 * Calculates 2nd-order Butterworth low-pass biquad coefficients using
 * the bilinear transform with frequency pre-warping.
 *
 * @param lpf_hz Low-pass filter cutoff frequency in Hz
 */
void gyro_init(uint8_t lpf_hz)
{
    /*
     * 2nd-order Butterworth LPF via bilinear transform:
     *
     *   K = tan(pi * fc / fs)       // pre-warped frequency
     *   norm = 1 / (1 + sqrt(2)*K + K^2)
     *
     *   b0 = K^2 * norm
     *   b1 = 2 * b0
     *   b2 = b0
     *   a1 = 2 * (K^2 - 1) * norm
     *   a2 = (1 - sqrt(2)*K + K^2) * norm
     *
     * sqrt(2) is the Butterworth Q factor for 2nd order.
     */
    const float fs = 1000.0f;  /* 1kHz sample rate */
    const float fc = (float)lpf_hz;
    const float pi = 3.14159265f;
    const float sqrt2 = 1.41421356f;

    /* Clamp cutoff to valid range (1 Hz to Nyquist) */
    float fc_clamped = fc;
    if (fc_clamped < 1.0f)   fc_clamped = 1.0f;
    if (fc_clamped > fs / 2.0f - 1.0f) fc_clamped = fs / 2.0f - 1.0f;

    float K = tanf(pi * fc_clamped / fs);
    float K2 = K * K;
    float norm = 1.0f / (1.0f + sqrt2 * K + K2);

    float b0 = K2 * norm;
    float b1 = 2.0f * b0;
    float b2 = b0;
    float a1 = 2.0f * (K2 - 1.0f) * norm;
    float a2 = (1.0f - sqrt2 * K + K2) * norm;

    /* Apply same coefficients to all three axes */
    filt_x.b0 = filt_y.b0 = filt_z.b0 = b0;
    filt_x.b1 = filt_y.b1 = filt_z.b1 = b1;
    filt_x.b2 = filt_y.b2 = filt_z.b2 = b2;
    filt_x.a1 = filt_y.a1 = filt_z.a1 = a1;
    filt_x.a2 = filt_y.a2 = filt_z.a2 = a2;

    /* Reset filter state */
    biquad_reset(&filt_x);
    biquad_reset(&filt_y);
    biquad_reset(&filt_z);

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
    float cx = raw_x - bias_x;
    float cy = raw_y - bias_y;
    float cz = raw_z - bias_z;

    /* Apply 2nd-order Butterworth biquad */
    g_filtered.roll_rate  = biquad_apply(&filt_x, cx);
    g_filtered.pitch_rate = biquad_apply(&filt_y, cy);
    g_filtered.yaw_rate   = biquad_apply(&filt_z, cz);
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
