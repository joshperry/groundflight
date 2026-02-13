/**
 * MSP (MultiWii Serial Protocol) Driver
 *
 * Implements MSP v1 frame building for DisplayPort OSD.
 * TX-only — we send OSD frames to the goggles, no parsing needed.
 */

#ifndef MSP_H
#define MSP_H

#include <stdint.h>
#include <stdbool.h>

/* MSP v1 frame structure:
 *   '$' 'M' '>' len cmd [payload...] crc
 *   crc = XOR of len, cmd, and all payload bytes
 */

/* MSP DisplayPort command */
#define MSP_DISPLAYPORT     182

/* DisplayPort sub-commands (first byte of payload) */
#define MSP_DP_HEARTBEAT    0   /* Keep link alive */
#define MSP_DP_RELEASE      0   /* Release display */
#define MSP_DP_CLEAR        1   /* Clear screen */
#define MSP_DP_WRITE        2   /* Write string: [row] [col] [attr] [chars...] */
#define MSP_DP_DRAW         3   /* Draw/commit screen */
#define MSP_DP_OPTIONS      4   /* Set options: [font] [rows] [cols_lo] [cols_hi] */

/* Display attributes */
#define MSP_DP_ATTR_NORMAL  0
#define MSP_DP_ATTR_INFO    1
#define MSP_DP_ATTR_WARN    2
#define MSP_DP_ATTR_CRIT    3

/* HD grid dimensions (DJI Goggles) */
#define MSP_OSD_COLS        50
#define MSP_OSD_ROWS        18

/* Maximum MSP frame size */
#define MSP_MAX_PAYLOAD     64
#define MSP_FRAME_OVERHEAD  6   /* $ M > len cmd crc */
#define MSP_MAX_FRAME       (MSP_MAX_PAYLOAD + MSP_FRAME_OVERHEAD)

/**
 * Initialize MSP driver on UART6 (Port B)
 *
 * @return true on success
 */
bool msp_init(void);

/**
 * Send a raw MSP frame
 *
 * @param cmd MSP command ID
 * @param payload Payload data (NULL if len=0)
 * @param len Payload length
 */
void msp_send(uint8_t cmd, const uint8_t *payload, uint8_t len);

/**
 * DisplayPort: clear screen
 */
void msp_dp_clear(void);

/**
 * DisplayPort: write string at position
 *
 * @param row Row (0-17)
 * @param col Column (0-49)
 * @param attr Display attribute (MSP_DP_ATTR_*)
 * @param str Null-terminated string
 */
void msp_dp_write(uint8_t row, uint8_t col, uint8_t attr, const char *str);

/**
 * DisplayPort: commit screen (draw)
 */
void msp_dp_draw(void);

/**
 * DisplayPort: send heartbeat
 */
void msp_dp_heartbeat(void);

/**
 * DisplayPort: set options (grid size, font)
 *
 * @param font Font index (0 = default)
 */
void msp_dp_options(uint8_t font);

#endif /* MSP_H */
