/**
 * OSD (On-Screen Display)
 *
 * Renders telemetry data to MSP DisplayPort for DJI goggles.
 * Telemetry at screen edges, branding in center when disarmed.
 */

#ifndef OSD_H
#define OSD_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize OSD subsystem (calls msp_init)
 *
 * @return true on success
 */
bool osd_init(void);

/**
 * Update OSD display
 *
 * Call from main loop. Rate-limits itself internally.
 *
 * @param now Current time in milliseconds
 */
void osd_update(uint32_t now);

/**
 * Set arm state (controls center branding display)
 *
 * @param is_armed true when vehicle is armed
 */
void osd_set_armed(bool is_armed);

#endif /* OSD_H */
