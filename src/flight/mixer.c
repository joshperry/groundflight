/**
 * Input/Output Mixer
 *
 * Maps CRSF inputs to servo outputs with stabilization
 *
 * The mixer combines user steering input with stabilizer correction
 * and ensures outputs stay within valid servo range.
 */

#include "mixer.h"

/* Mixed output values */
static mixer_output_t g_output = {0};

/**
 * Initialize mixer
 */
void mixer_init(void)
{
    /* Start with neutral values */
    g_output.steer = 0.0f;
    g_output.throttle = 0.0f;
    g_output.ebrake = 0.0f;
    g_output.aux = 0.0f;
}

/**
 * Update mixer with new inputs
 *
 * @param steer_in Steering command from receiver (-1.0 to 1.0)
 * @param throttle_in Throttle command from receiver (-1.0 to 1.0)
 * @param ebrake_in E-brake command from receiver (0.0 to 1.0)
 * @param aux_in Aux channel from receiver (-1.0 to 1.0)
 * @param stab_correction Steering correction from stabilizer (-1.0 to 1.0)
 */
void mixer_update(float steer_in, float throttle_in, float ebrake_in,
                  float aux_in, float stab_correction)
{
    /* Combine steering input with stabilizer correction
     *
     * Simple additive mixing:
     *   output = input + correction
     *
     * The stabilizer already limits its correction to max_correction,
     * but we still need to clamp the final output to valid servo range.
     */
    float steer_mixed = steer_in + stab_correction;

    /* Clamp steering to valid range */
    if (steer_mixed > 1.0f) {
        steer_mixed = 1.0f;
    } else if (steer_mixed < -1.0f) {
        steer_mixed = -1.0f;
    }

    /* Throttle, e-brake, and aux are passthrough (no stabilization) */

    /* Clamp throttle to valid range */
    if (throttle_in > 1.0f) {
        throttle_in = 1.0f;
    } else if (throttle_in < -1.0f) {
        throttle_in = -1.0f;
    }

    /* Clamp e-brake to valid range (0.0 to 1.0, not bipolar) */
    if (ebrake_in > 1.0f) {
        ebrake_in = 1.0f;
    } else if (ebrake_in < 0.0f) {
        ebrake_in = 0.0f;
    }

    /* Clamp aux to valid range */
    if (aux_in > 1.0f) {
        aux_in = 1.0f;
    } else if (aux_in < -1.0f) {
        aux_in = -1.0f;
    }

    /* Update output structure */
    g_output.steer = steer_mixed;
    g_output.throttle = throttle_in;
    g_output.ebrake = ebrake_in;
    g_output.aux = aux_in;
}

/**
 * Get mixer output
 *
 * @return Pointer to mixer output (read-only)
 */
const mixer_output_t* mixer_get_output(void)
{
    return &g_output;
}
