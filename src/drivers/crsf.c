/**
 * CRSF (Crossfire) Protocol Parser
 * 
 * State machine that parses CRSF frames from UART ring buffer.
 */

#include "crsf.h"
#include "uart.h"
#include "target.h"
#include <string.h>

/* Parser state machine */
typedef enum {
    CRSF_STATE_WAIT_SYNC,
    CRSF_STATE_GOT_SYNC,
    CRSF_STATE_GOT_LEN,
    CRSF_STATE_RECEIVING,
} crsf_parser_state_t;

/* Parser context */
static struct {
    crsf_parser_state_t state;
    uint8_t  frame[CRSF_FRAME_SIZE_MAX];
    uint8_t  frame_len;      /* Expected frame length (from len byte) */
    uint8_t  frame_pos;      /* Current position in frame */
    uint32_t last_byte_ms;   /* Timestamp of last byte (for timeout) */
} parser;

/* Parsed data */
static crsf_state_t crsf_state;

/* Failsafe timeout (ms) */
#define CRSF_FAILSAFE_TIMEOUT_MS  500

/* Frame timeout - reset parser if no bytes for this long */
#define CRSF_FRAME_TIMEOUT_MS     5

/* CRC8 lookup table (polynomial 0xD5) - same as xbox-elrs */
static const uint8_t crc8_lut[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
};

/**
 * Calculate CRC8 over buffer (poly 0xD5)
 */
static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = crc8_lut[crc ^ data[i]];
    }
    return crc;
}

/**
 * Unpack 16 channels from 22-byte packed format
 * 
 * This is the inverse of pack_channels() in xbox-elrs.
 * 16 channels × 11 bits = 176 bits = 22 bytes
 */
static void unpack_channels(const uint8_t *packed, uint16_t *channels)
{
    channels[0]  = ((packed[0]       | (packed[1]  << 8)) & 0x07FF);
    channels[1]  = ((packed[1]  >> 3 | (packed[2]  << 5)) & 0x07FF);
    channels[2]  = ((packed[2]  >> 6 | (packed[3]  << 2) | (packed[4] << 10)) & 0x07FF);
    channels[3]  = ((packed[4]  >> 1 | (packed[5]  << 7)) & 0x07FF);
    channels[4]  = ((packed[5]  >> 4 | (packed[6]  << 4)) & 0x07FF);
    channels[5]  = ((packed[6]  >> 7 | (packed[7]  << 1) | (packed[8] << 9)) & 0x07FF);
    channels[6]  = ((packed[8]  >> 2 | (packed[9]  << 6)) & 0x07FF);
    channels[7]  = ((packed[9]  >> 5 | (packed[10] << 3)) & 0x07FF);
    channels[8]  = ((packed[11]      | (packed[12] << 8)) & 0x07FF);
    channels[9]  = ((packed[12] >> 3 | (packed[13] << 5)) & 0x07FF);
    channels[10] = ((packed[13] >> 6 | (packed[14] << 2) | (packed[15] << 10)) & 0x07FF);
    channels[11] = ((packed[15] >> 1 | (packed[16] << 7)) & 0x07FF);
    channels[12] = ((packed[16] >> 4 | (packed[17] << 4)) & 0x07FF);
    channels[13] = ((packed[17] >> 7 | (packed[18] << 1) | (packed[19] << 9)) & 0x07FF);
    channels[14] = ((packed[19] >> 2 | (packed[20] << 6)) & 0x07FF);
    channels[15] = ((packed[20] >> 5 | (packed[21] << 3)) & 0x07FF);
}

/**
 * Process a complete, validated frame
 */
static void process_frame(void)
{
    uint8_t type = parser.frame[2];
    uint8_t *payload = &parser.frame[3];
    
    switch (type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            /* 22 bytes of packed channel data */
            unpack_channels(payload, crsf_state.channels);
            crsf_state.last_rc_frame_ms = target_millis();
            crsf_state.failsafe = false;
            break;
            
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            /* 10 bytes of link stats */
            crsf_state.link.uplink_rssi_1 = payload[0];
            crsf_state.link.uplink_rssi_2 = payload[1];
            crsf_state.link.uplink_link_quality = payload[2];
            crsf_state.link.uplink_snr = (int8_t)payload[3];
            crsf_state.link.active_antenna = payload[4];
            crsf_state.link.rf_mode = payload[5];
            crsf_state.link.uplink_tx_power = payload[6];
            crsf_state.link.downlink_rssi = payload[7];
            crsf_state.link.downlink_link_quality = payload[8];
            crsf_state.link.downlink_snr = (int8_t)payload[9];
            crsf_state.last_link_frame_ms = target_millis();
            break;
            
        default:
            /* Ignore other frame types */
            break;
    }
    
    crsf_state.frame_count++;
}

/**
 * Reset parser state
 */
static void parser_reset(void)
{
    parser.state = CRSF_STATE_WAIT_SYNC;
    parser.frame_pos = 0;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

bool crsf_init(void)
{
    /* Initialize state */
    memset(&crsf_state, 0, sizeof(crsf_state));
    memset(&parser, 0, sizeof(parser));
    
    /* Set default channel values (center/safe) */
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        crsf_state.channels[i] = CRSF_CHANNEL_MID;
    }
    
    crsf_state.failsafe = true;  /* Start in failsafe until we get data */
    parser.state = CRSF_STATE_WAIT_SYNC;
    
    /* Initialize UART4 at CRSF baud rate */
    return uart_init(UART_CRSF, CRSF_BAUDRATE);
}

void crsf_process(void)
{
    uint32_t now = target_millis();
    int16_t byte;
    
    /* Check for frame timeout (inter-byte gap too long) */
    if (parser.state != CRSF_STATE_WAIT_SYNC) {
        if ((now - parser.last_byte_ms) > CRSF_FRAME_TIMEOUT_MS) {
            parser_reset();
        }
    }
    
    /* Check for failsafe timeout */
    if (!crsf_state.failsafe && 
        (now - crsf_state.last_rc_frame_ms) > CRSF_FAILSAFE_TIMEOUT_MS) {
        crsf_state.failsafe = true;
    }
    
    /* Process all available bytes */
    while ((byte = uart_read_byte(UART_CRSF)) >= 0) {
        uint8_t data = (uint8_t)byte;
        parser.last_byte_ms = now;
        
        switch (parser.state) {
            case CRSF_STATE_WAIT_SYNC:
                if (data == CRSF_SYNC_BYTE) {
                    parser.frame[0] = data;
                    parser.frame_pos = 1;
                    parser.state = CRSF_STATE_GOT_SYNC;
                }
                break;
                
            case CRSF_STATE_GOT_SYNC:
                /* Length byte: number of bytes following (type + payload + crc) */
                if (data >= 2 && data <= CRSF_PAYLOAD_SIZE_MAX + 2) {
                    parser.frame[1] = data;
                    parser.frame_len = data + 2;  /* Total frame size including sync and len */
                    parser.frame_pos = 2;
                    parser.state = CRSF_STATE_RECEIVING;
                } else {
                    /* Invalid length, reset */
                    parser_reset();
                }
                break;
                
            case CRSF_STATE_RECEIVING:
                parser.frame[parser.frame_pos++] = data;
                
                if (parser.frame_pos >= parser.frame_len) {
                    /* Frame complete - validate CRC */
                    /* CRC is over type + payload (bytes 2 to frame_len-2) */
                    uint8_t expected_crc = parser.frame[parser.frame_len - 1];
                    uint8_t calc_crc = crc8(&parser.frame[2], parser.frame_len - 3);
                    
                    if (calc_crc == expected_crc) {
                        process_frame();
                    } else {
                        crsf_state.error_count++;
                    }
                    
                    parser_reset();
                }
                break;
                
            default:
                parser_reset();
                break;
        }
    }
}

const crsf_state_t* crsf_get_state(void)
{
    return &crsf_state;
}

bool crsf_is_connected(void)
{
    return !crsf_state.failsafe && (crsf_state.frame_count > 0);
}
