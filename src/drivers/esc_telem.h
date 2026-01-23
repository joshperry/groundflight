/**
 * ESC Telemetry Abstraction
 * 
 * Provides unified interface regardless of telemetry protocol
 */

#ifndef ESC_TELEM_H
#define ESC_TELEM_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t rpm;           /* Motor electrical RPM */
    float    voltage;       /* Pack voltage */
    float    current;       /* Motor current (amps) */
    float    temperature;   /* ESC temp (°C) */
    uint32_t mah_consumed;  /* Capacity used */
    uint32_t last_update_ms;
    bool     valid;
} esc_telemetry_t;

typedef enum {
    ESC_PROTO_NONE,
    ESC_PROTO_SRXL2,
    ESC_PROTO_KISS,
    ESC_PROTO_HOBBYWING,
    ESC_PROTO_CASTLE,
} esc_protocol_t;

void  esc_init(esc_protocol_t proto);
void  esc_update(void);
float esc_get_speed_mph(void);
bool  esc_telemetry_valid(void);
const esc_telemetry_t* esc_get_telemetry(void);

#endif /* ESC_TELEM_H */
