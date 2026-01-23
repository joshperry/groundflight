/**
 * No-Telemetry ESC Fallback
 */

#ifndef ESC_NONE_H
#define ESC_NONE_H

#include "esc_telem.h"

bool esc_none_init(void);
const esc_telemetry_t* esc_none_get_telemetry(void);

#endif /* ESC_NONE_H */
