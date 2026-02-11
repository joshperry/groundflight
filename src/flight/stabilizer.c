/**
 * Yaw Stabilizer
 *
 * Core control algorithm for car stability
 *
 * Algorithm (MVP P-only):
 *   expected_yaw_rate = steer_cmd * yaw_rate_scale
 *   error = expected_yaw_rate - gyro_yaw
 *   correction = Kp * error * gain_knob
 *   servo_out = steer_cmd + correction (mixer handles this)
 */

#include "stabilizer.h"

/* Current configuration */
static stab_gains_t g_gains = {0};

/* Current mode */
static stab_mode_t g_mode = STAB_MODE_OFF;

/* Last correction output (for debugging/telemetry) */
static float g_last_correction = 0.0f;
static float g_last_error = 0.0f;

/**
 * Initialize stabilizer
 *
 * @param gains Pointer to gain configuration
 */
void stabilizer_init(const stab_gains_t *gains)
{
    if (gains) {
        g_gains = *gains;
    }

    g_mode = STAB_MODE_OFF;
    g_last_correction = 0.0f;
    g_last_error = 0.0f;
}

/**
 * Set stabilizer mode
 *
 * @param mode Stabilizer mode (off/normal/sport)
 */
void stabilizer_set_mode(stab_mode_t mode)
{
    g_mode = mode;

    /* Reset state when changing modes */
    if (mode == STAB_MODE_OFF) {
        g_last_correction = 0.0f;
        g_last_error = 0.0f;
    }
}

/**
 * Update stabilizer (main control loop)
 *
 * @param steer_cmd Steering command from receiver (-1.0 to 1.0)
 * @param gyro_yaw Current yaw rate from gyro (deg/s)
 * @param speed_mph Current vehicle speed in mph (unused in MVP)
 * @param throttle Throttle position (-1.0 to 1.0, unused in MVP)
 * @param gain_knob Gain adjustment knob (0.0 to 1.0, 1.0 = full gain)
 *
 * @return Steering correction to add to steer_cmd (-1.0 to 1.0)
 */
float stabilizer_update(float steer_cmd, float gyro_yaw, float speed_mph,
                        float throttle, float gain_knob)
{
    /* If disabled, return zero correction */
    if (g_mode == STAB_MODE_OFF) {
        g_last_correction = 0.0f;
        g_last_error = 0.0f;
        return 0.0f;
    }

    /* Calculate expected yaw rate from steering command
     *
     * steer_cmd is -1.0 (full left) to 1.0 (full right)
     * yaw_rate_scale defines how many deg/s we expect at full stick
     *
     * Example: yaw_rate_scale = 400 deg/s
     *   Full right stick (1.0) → expect +400 deg/s yaw rate
     *   Centered (0.0) → expect 0 deg/s (straight line)
     *   Full left (-1.0) → expect -400 deg/s yaw rate
     */
    float expected_yaw_rate = steer_cmd * g_gains.yaw_rate_scale;

    /* Calculate yaw rate error
     *
     * Positive error means we're not turning fast enough
     * Negative error means we're turning too fast (oversteer)
     */
    float error = expected_yaw_rate - gyro_yaw;

    /* Apply proportional gain
     *
     * For MVP: Simple P-only controller
     * correction = Kp * error
     */
    float correction = g_gains.kp * error;

    /* Apply gain knob (user-adjustable trim)
     *
     * gain_knob range: 0.0 (no correction) to 1.0 (full correction)
     * Defaults to 1.0 if not used
     */
    if (gain_knob < 0.0f) gain_knob = 0.0f;
    if (gain_knob > 1.0f) gain_knob = 1.0f;
    correction *= gain_knob;

    /* Apply correction limits
     *
     * max_correction is in normalized units (-1.0 to 1.0)
     * Prevents stabilizer from saturating the servo
     */
    float max_corr = g_gains.max_correction;
    if (correction > max_corr) {
        correction = max_corr;
    } else if (correction < -max_corr) {
        correction = -max_corr;
    }

    /* Store for debugging/telemetry */
    g_last_correction = correction;
    g_last_error = error;

    return correction;
}
