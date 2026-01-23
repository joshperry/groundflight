/**
 * Yaw Stabilizer
 */

#ifndef STABILIZER_H
#define STABILIZER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STAB_MODE_OFF = 0,
    STAB_MODE_NORMAL,
    STAB_MODE_SPORT,
} stab_mode_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float yaw_rate_scale;
    float max_correction;
} stab_gains_t;

void stabilizer_init(const stab_gains_t *gains);
void stabilizer_set_mode(stab_mode_t mode);
float stabilizer_update(float steer_cmd, float gyro_yaw, float speed_mph, 
                        float throttle, float gain_knob);

#endif /* STABILIZER_H */
