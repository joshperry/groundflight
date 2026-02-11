/**
 * Speed Estimation
 *
 * Calculate vehicle speed from ESC telemetry
 */

#include "speed.h"

/* Vehicle parameters */
static float g_gear_ratio = 1.0f;
static float g_tire_circumference_m = 0.0f;  /* meters */
static uint8_t g_motor_pole_pairs = 1;

/* Current speed */
static float g_speed_mph = 0.0f;
static float g_speed_mps = 0.0f;

/**
 * Initialize speed estimator
 *
 * @param gear_ratio Gear reduction ratio (e.g., 7.7 for 7.7:1)
 * @param tire_diameter_mm Tire diameter in millimeters
 * @param motor_poles Number of motor poles (not pole pairs!)
 */
void speed_init(float gear_ratio, uint16_t tire_diameter_mm, uint8_t motor_poles)
{
    g_gear_ratio = gear_ratio;
    g_motor_pole_pairs = motor_poles / 2;  /* Pole pairs = poles / 2 */
    if (g_motor_pole_pairs < 1) g_motor_pole_pairs = 1;

    /* Calculate tire circumference in meters */
    const float pi = 3.14159265f;
    g_tire_circumference_m = (float)tire_diameter_mm / 1000.0f * pi;
}

/**
 * Update speed estimate from motor RPM
 *
 * @param motor_rpm Motor electrical RPM from ESC telemetry
 *
 * Conversion chain:
 *   1. Electrical RPM → Mechanical RPM (divide by pole pairs)
 *   2. Motor RPM → Wheel RPM (divide by gear ratio)
 *   3. Wheel RPM → m/s (multiply by circumference / 60)
 *   4. m/s → mph (multiply by 2.237)
 */
void speed_update(uint32_t motor_rpm)
{
    if (motor_rpm == 0) {
        g_speed_mph = 0.0f;
        g_speed_mps = 0.0f;
        return;
    }

    /* Convert electrical RPM to mechanical RPM */
    float mechanical_rpm = (float)motor_rpm / (float)g_motor_pole_pairs;

    /* Convert motor RPM to wheel RPM */
    float wheel_rpm = mechanical_rpm / g_gear_ratio;

    /* Convert wheel RPM to meters per second
     * wheel_rpm = revolutions per minute
     * speed (m/s) = (wheel_rpm / 60) * circumference
     */
    g_speed_mps = (wheel_rpm / 60.0f) * g_tire_circumference_m;

    /* Convert m/s to mph
     * 1 m/s = 2.237 mph
     */
    g_speed_mph = g_speed_mps * 2.237f;
}

/**
 * Get current speed in mph
 *
 * @return Vehicle speed in miles per hour
 */
float speed_get_mph(void)
{
    return g_speed_mph;
}

/**
 * Get current speed in m/s
 *
 * @return Vehicle speed in meters per second
 */
float speed_get_mps(void)
{
    return g_speed_mps;
}
