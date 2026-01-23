/**
 * SRXL2 ESC Telemetry Backend
 */

#ifndef ESC_SRXL2_H
#define ESC_SRXL2_H

#include "esc_telem.h"

bool esc_srxl2_init(void);
void esc_srxl2_update(void);
const esc_telemetry_t* esc_srxl2_get_telemetry(void);

#endif /* ESC_SRXL2_H */
