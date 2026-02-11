/**
 * Runtime Configuration
 *
 * Manages all tunable parameters
 */

#include "config.h"
#include <string.h>

/* Global configuration instance */
static config_t g_config;

/* Default configuration values (MVP - tuned for Arrma Infraction 6S) */
static const config_t DEFAULT_CONFIG = {
    /* Gyro */
    .gyro_lpf_hz = 50,              /* 50Hz low-pass filter */

    /* Stabilizer PID - MVP: P-only, conservative gain */
    .kp = 0.003f,                   /* Correction per deg/s error (0.003 = 50% at 167 dps) */
    .ki = 0.0f,                     /* Disabled for MVP */
    .kd = 0.0f,                     /* Disabled for MVP */
    .yaw_rate_scale = 400.0f,       /* Full stick = 400 deg/s expected yaw rate */
    .max_correction = 0.5f,         /* Limit correction to ±50% of servo range */

    /* Speed-based gain scheduling (not used in MVP, placeholder) */
    .low_speed_gain = 1.0f,
    .high_speed_gain = 1.0f,
    .speed_gain_max_mph = 40.0f,

    /* Brake-aware gain (not used in MVP, placeholder) */
    .brake_gain_max = 1.0f,
    .trail_brake_steer_threshold = 0.3f,
    .trail_brake_reduction = 0.5f,

    /* Steering servo - standard RC range */
    .steer_center_us = 1500,        /* Center position */
    .steer_range_us = 400,          /* ±400us range (1100-1900) */
    .steer_direction = 1,           /* 1 = normal, -1 = reversed */

    /* E-brake servo */
    .ebrake_min_us = 1000,
    .ebrake_max_us = 2000,

    /* ESC - assuming SRXL2 mode with Spektrum Firma 150A */
    .esc_protocol = 1,              /* 0=PWM, 1=SRXL2 */
    .esc_min_us = 1000,
    .esc_max_us = 2000,
    .esc_center_us = 1500,

    /* Vehicle geometry - Arrma Infraction 6S typical values */
    .gear_ratio = 7.7f,             /* 7.7:1 gear ratio (basher setup) */
    .tire_diameter_mm = 107,        /* 107mm diameter (street basher wheels) */
    .motor_poles = 4,               /* 4-pole motor (2 pole pairs) */

    /* Channel mapping (matching current passthrough in main.c) */
    .ch_steering = 0,               /* CH1 = steering */
    .ch_throttle = 2,               /* CH3 = throttle */
    .ch_gain = 5,                   /* CH6 = gain knob (future use) */
    .ch_mode = 6,                   /* CH7 = mode switch (future use) */
    .ch_ebrake = 3,                 /* CH4 = e-brake */

    /* Modes */
    .mode_count = 2,                /* 0=off, 1=normal (sport mode future) */
};

void config_init(void)
{
    /* Load defaults */
    config_reset_defaults();
}

void config_load(void)
{
    /* TODO: Load from EEPROM/flash
     * For MVP, just use defaults
     */
    config_reset_defaults();
}

void config_save(void)
{
    /* TODO: Save to EEPROM/flash
     * For MVP, this is a no-op
     */
}

void config_reset_defaults(void)
{
    memcpy(&g_config, &DEFAULT_CONFIG, sizeof(config_t));
}

config_t* config_get(void)
{
    return &g_config;
}
