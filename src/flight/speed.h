/**
 * Speed Estimation
 */

#ifndef SPEED_H
#define SPEED_H

#include <stdint.h>

void speed_init(float gear_ratio, uint16_t tire_diameter_mm, uint8_t motor_poles);
void speed_update(uint32_t motor_rpm);
float speed_get_mph(void);
float speed_get_mps(void);

#endif /* SPEED_H */
