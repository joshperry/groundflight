/**
 * OSD (On-Screen Display)
 *
 * Renders telemetry to MSP DisplayPort for DJI goggles.
 *
 * Layout (50x18 HD grid):
 *
 *   Row 0:  Speed (left)                    Voltage (right)
 *   Row 1:  Stab mode (left)               Current (right)
 *   Row 2:  Yaw rate (left)                Temp (right)
 *   Row 3:  Correction (left)              Gain (right)
 *   Row 4:  Steer trim (left)
 *
 *   Rows 7-10 (disarmed only):
 *           GROUNDFLIGHT
 *           v0.1.0
 *           [status line]
 *           Flip CH5 to ARM
 *
 *   Row 17: RPM (left)                     ARM state (right)
 */

#include "osd.h"
#include "msp.h"
#include "esc.h"
#include "speed.h"
#include "gyro.h"
#include "stabilizer.h"
#include "config.h"
#include "crsf.h"
#include <string.h>

/* OSD update rate */
#define OSD_UPDATE_INTERVAL_MS  50  /* 20 Hz */

/* State */
static bool g_initialized = false;
static bool g_armed = false;
static uint32_t g_last_update = 0;

/* Right-align helper: write string ending at column */
static void osd_write_right(uint8_t row, uint8_t end_col, uint8_t attr, const char *str)
{
    uint8_t len = 0;
    const char *p = str;
    while (*p++) len++;
    uint8_t col = (end_col >= len) ? (end_col - len) : 0;
    msp_dp_write(row, col, attr, str);
}

/* Format integer into buffer, returns length written */
static uint8_t fmt_int(char *buf, uint8_t size, int32_t val)
{
    char tmp[12];
    uint8_t i = 0;
    bool neg = false;

    if (val < 0) {
        neg = true;
        val = -val;
    }

    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0 && i < sizeof(tmp)) {
            tmp[i++] = '0' + (val % 10);
            val /= 10;
        }
    }

    uint8_t pos = 0;
    if (neg && pos < size - 1) buf[pos++] = '-';
    while (i > 0 && pos < size - 1) {
        buf[pos++] = tmp[--i];
    }
    buf[pos] = '\0';
    return pos;
}

/* Format float with 1 decimal place */
static void fmt_float1(char *buf, uint8_t size, float val)
{
    bool neg = false;
    if (val < 0.0f) {
        neg = true;
        val = -val;
    }

    int32_t int_part = (int32_t)val;
    int32_t frac = (int32_t)((val - (float)int_part) * 10.0f + 0.5f);
    if (frac >= 10) {
        int_part++;
        frac = 0;
    }

    uint8_t pos = 0;
    if (neg && pos < size - 1) buf[pos++] = '-';

    /* Integer part */
    char tmp[12];
    uint8_t tlen = fmt_int(tmp, sizeof(tmp), int_part);
    for (uint8_t i = 0; i < tlen && pos < size - 1; i++) {
        buf[pos++] = tmp[i];
    }

    if (pos < size - 2) {
        buf[pos++] = '.';
        buf[pos++] = '0' + (uint8_t)frac;
    }
    buf[pos] = '\0';
}

bool osd_init(void)
{
    if (!msp_init()) {
        return false;
    }
    g_initialized = true;
    return true;
}

void osd_set_armed(bool is_armed)
{
    g_armed = is_armed;
}

void osd_update(uint32_t now)
{
    if (!g_initialized) return;

    /* Rate limit */
    if ((now - g_last_update) < OSD_UPDATE_INTERVAL_MS) return;
    g_last_update = now;

    char buf[24];
    const uint8_t RIGHT_COL = 49;  /* Right edge of 50-col grid */

    /* Clear screen */
    msp_dp_clear();

    /* ---- Top left: Speed ---- */
    float mph = speed_get_mph();
    fmt_float1(buf, sizeof(buf), mph);
    {
        uint8_t len = 0;
        while (buf[len]) len++;
        if (len < sizeof(buf) - 4) {
            buf[len++] = ' ';
            buf[len++] = 'M';
            buf[len++] = 'P';
            buf[len++] = 'H';
            buf[len] = '\0';
        }
    }
    msp_dp_write(0, 0, MSP_DP_ATTR_NORMAL, buf);

    /* ---- Top right: Voltage ---- */
    if (esc_telemetry_valid()) {
        const esc_telemetry_t *telem = esc_get_telemetry();
        fmt_float1(buf, sizeof(buf), telem->voltage);
        uint8_t len = 0;
        while (buf[len]) len++;
        if (len < sizeof(buf) - 1) {
            buf[len++] = 'V';
            buf[len] = '\0';
        }
        osd_write_right(0, RIGHT_COL, MSP_DP_ATTR_NORMAL, buf);
    }

    /* ---- Row 1 left: Stabilizer mode ---- */
    {
        const char *mode_str = "STAB:OFF";
        if (g_armed) {
            mode_str = "STAB:ON";
        }
        msp_dp_write(1, 0, MSP_DP_ATTR_NORMAL, mode_str);
    }

    /* ---- Row 1 right: Current ---- */
    if (esc_telemetry_valid()) {
        const esc_telemetry_t *telem = esc_get_telemetry();
        fmt_float1(buf, sizeof(buf), telem->current);
        uint8_t len = 0;
        while (buf[len]) len++;
        if (len < sizeof(buf) - 1) {
            buf[len++] = 'A';
            buf[len] = '\0';
        }
        osd_write_right(1, RIGHT_COL, MSP_DP_ATTR_NORMAL, buf);
    }

    /* ---- Row 2 left: Yaw rate ---- */
    {
        const gyro_filtered_t *gyro = gyro_get_filtered();
        fmt_int(buf, sizeof(buf), (int32_t)gyro->yaw_rate);
        uint8_t len = 0;
        while (buf[len]) len++;
        if (len < sizeof(buf) - 4) {
            buf[len++] = ' ';
            buf[len++] = 'D';
            buf[len++] = '/';
            buf[len++] = 'S';
            buf[len] = '\0';
        }
        msp_dp_write(2, 0, MSP_DP_ATTR_NORMAL, buf);
    }

    /* ---- Row 2 right: ESC temp ---- */
    if (esc_telemetry_valid()) {
        const esc_telemetry_t *telem = esc_get_telemetry();
        fmt_int(buf, sizeof(buf), (int32_t)telem->temperature);
        uint8_t len = 0;
        while (buf[len]) len++;
        if (len < sizeof(buf) - 1) {
            buf[len++] = 'C';
            buf[len] = '\0';
        }
        osd_write_right(2, RIGHT_COL, MSP_DP_ATTR_NORMAL, buf);
    }

    /* ---- Row 3 left: Steering correction ---- */
    {
        /* stabilizer exposes last correction via update with OFF mode check,
         * but we need a getter. For now show gyro Z filtered. */
        const gyro_filtered_t *gyro = gyro_get_filtered();
        /* Show filtered yaw as proxy for correction activity */
        (void)gyro;
        msp_dp_write(3, 0, MSP_DP_ATTR_NORMAL, "CORR");
    }

    /* ---- Row 3 right: Gain ---- */
    {
        config_t *cfg = config_get();
        /* Show Kp as gain indicator */
        fmt_float1(buf, sizeof(buf), cfg->kp * 1000.0f);
        uint8_t len = 0;
        while (buf[len]) len++;
        /* Append label */
        const char *suffix = " KP";
        for (uint8_t i = 0; suffix[i] && len < sizeof(buf) - 1; i++) {
            buf[len++] = suffix[i];
        }
        buf[len] = '\0';
        osd_write_right(3, RIGHT_COL, MSP_DP_ATTR_NORMAL, buf);
    }

    /* ---- Row 4 left: Steering trim ---- */
    {
        config_t *cfg = config_get();
        /* Show steering center offset from 1500 */
        int32_t trim = (int32_t)cfg->steer_center_us - 1500;
        if (trim != 0) {
            buf[0] = 'T';
            buf[1] = 'R';
            buf[2] = 'I';
            buf[3] = 'M';
            buf[4] = ' ';
            fmt_int(buf + 5, sizeof(buf) - 5, trim);
            msp_dp_write(4, 0, MSP_DP_ATTR_NORMAL, buf);
        }
    }

    /* ---- Center: Branding when disarmed ---- */
    if (!g_armed) {
        msp_dp_write(7,  17, MSP_DP_ATTR_INFO, "GROUNDFLIGHT");
        msp_dp_write(8,  20, MSP_DP_ATTR_NORMAL, "v0.1.0");
        msp_dp_write(10, 15, MSP_DP_ATTR_WARN, "Flip CH5 to ARM");
    }

    /* ---- Bottom left: RPM ---- */
    if (esc_telemetry_valid()) {
        const esc_telemetry_t *telem = esc_get_telemetry();
        fmt_int(buf, sizeof(buf), (int32_t)telem->rpm);
        uint8_t len = 0;
        while (buf[len]) len++;
        if (len < sizeof(buf) - 4) {
            buf[len++] = ' ';
            buf[len++] = 'R';
            buf[len++] = 'P';
            buf[len++] = 'M';
            buf[len] = '\0';
        }
        msp_dp_write(17, 0, MSP_DP_ATTR_NORMAL, buf);
    }

    /* ---- Bottom right: ARM state ---- */
    if (g_armed) {
        osd_write_right(17, RIGHT_COL, MSP_DP_ATTR_CRIT, "ARMED");
    } else {
        osd_write_right(17, RIGHT_COL, MSP_DP_ATTR_NORMAL, "DISARMED");
    }

    /* Commit frame */
    msp_dp_draw();
}
