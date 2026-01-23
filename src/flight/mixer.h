/**
 * Input/Output Mixer
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float steer;    /* -1.0 to 1.0 */
    float throttle; /* -1.0 to 1.0 */
    float ebrake;   /*  0.0 to 1.0 */
    float aux;      /* -1.0 to 1.0 */
} mixer_output_t;

void mixer_init(void);
void mixer_update(float steer_in, float throttle_in, float ebrake_in, 
                  float aux_in, float stab_correction);
const mixer_output_t* mixer_get_output(void);

#endif /* MIXER_H */
