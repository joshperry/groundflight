/**
 * Gyro Signal Processing
 */

#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float yaw_rate;     /* deg/s, filtered */
    float pitch_rate;   /* deg/s, filtered */
    float roll_rate;    /* deg/s, filtered */
} gyro_filtered_t;

void gyro_init(uint8_t lpf_hz);
void gyro_update(float raw_x, float raw_y, float raw_z);
void gyro_calibrate(void);
const gyro_filtered_t* gyro_get_filtered(void);

#endif /* GYRO_H */
