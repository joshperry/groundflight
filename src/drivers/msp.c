/**
 * MSP (MultiWii Serial Protocol) Driver
 *
 * TX-only MSP v1 implementation for DisplayPort OSD.
 * Sends frames on UART6 (Port B, PC6/PC7) at 115200 baud.
 */

#include "msp.h"
#include "uart.h"
#include <string.h>

static bool g_initialized = false;

/* Build and send an MSP v1 frame */
static void msp_send_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[MSP_MAX_FRAME];
    uint8_t pos = 0;

    /* Header */
    frame[pos++] = '$';
    frame[pos++] = 'M';
    frame[pos++] = '>';

    /* Length and command */
    frame[pos++] = len;
    frame[pos++] = cmd;

    /* CRC starts with len ^ cmd */
    uint8_t crc = len ^ cmd;

    /* Payload */
    for (uint8_t i = 0; i < len && i < MSP_MAX_PAYLOAD; i++) {
        frame[pos++] = payload[i];
        crc ^= payload[i];
    }

    /* CRC */
    frame[pos++] = crc;

    uart_send(UART_AUX, frame, pos);
}

bool msp_init(void)
{
    if (!uart_init(UART_AUX, 115200)) {
        return false;
    }

    g_initialized = true;

    /* Configure DisplayPort grid size */
    msp_dp_options(0);

    return true;
}

void msp_send(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (!g_initialized) return;
    msp_send_frame(cmd, payload, len);
}

void msp_dp_clear(void)
{
    uint8_t payload[] = { MSP_DP_CLEAR };
    msp_send(MSP_DISPLAYPORT, payload, sizeof(payload));
}

void msp_dp_write(uint8_t row, uint8_t col, uint8_t attr, const char *str)
{
    uint8_t payload[MSP_MAX_PAYLOAD];
    uint8_t len = 0;

    payload[len++] = MSP_DP_WRITE;
    payload[len++] = row;
    payload[len++] = col;
    payload[len++] = attr;

    /* Copy string, truncate if too long */
    while (*str && len < MSP_MAX_PAYLOAD) {
        payload[len++] = (uint8_t)*str++;
    }

    msp_send(MSP_DISPLAYPORT, payload, len);
}

void msp_dp_draw(void)
{
    uint8_t payload[] = { MSP_DP_DRAW };
    msp_send(MSP_DISPLAYPORT, payload, sizeof(payload));
}

void msp_dp_heartbeat(void)
{
    uint8_t payload[] = { MSP_DP_HEARTBEAT };
    msp_send(MSP_DISPLAYPORT, payload, sizeof(payload));
}

void msp_dp_options(uint8_t font)
{
    uint8_t payload[5];
    payload[0] = MSP_DP_OPTIONS;
    payload[1] = font;
    payload[2] = MSP_OSD_ROWS;
    payload[3] = MSP_OSD_COLS & 0xFF;
    payload[4] = (MSP_OSD_COLS >> 8) & 0xFF;
    msp_send(MSP_DISPLAYPORT, payload, sizeof(payload));
}
