/**
 * Runtime Configuration
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* Gyro */
    uint8_t  gyro_lpf_hz;
    
    /* Stabilizer PID */
    float    kp;
    float    ki;
    float    kd;
    float    yaw_rate_scale;
    float    max_correction;
    
    /* Speed-based gain scheduling */
    float    low_speed_gain;
    float    high_speed_gain;
    float    speed_gain_max_mph;
    
    /* Brake-aware gain */
    float    brake_gain_max;
    float    trail_brake_steer_threshold;
    float    trail_brake_reduction;
    
    /* Steering servo */
    uint16_t steer_center_us;
    uint16_t steer_range_us;
    int8_t   steer_direction;
    
    /* E-brake servo */
    uint16_t ebrake_min_us;
    uint16_t ebrake_max_us;
    
    /* ESC */
    uint8_t  esc_protocol;
    uint16_t esc_min_us;
    uint16_t esc_max_us;
    uint16_t esc_center_us;
    
    /* Vehicle geometry */
    float    gear_ratio;
    uint16_t tire_diameter_mm;
    uint8_t  motor_poles;
    
    /* Channel mapping */
    uint8_t  ch_steering;
    uint8_t  ch_throttle;
    uint8_t  ch_gain;
    uint8_t  ch_mode;
    uint8_t  ch_ebrake;
    
    /* Modes */
    uint8_t  mode_count;
} config_t;

void config_init(void);
void config_load(void);
void config_save(void);
void config_reset_defaults(void);
config_t* config_get(void);

#endif /* CONFIG_H */
