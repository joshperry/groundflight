/**
 * SRXL2 Protocol Driver for GroundFlight
 * 
 * Implements bus master role (Flight Controller) to communicate with
 * Spektrum Smart ESC. Based on public SRXL2 specification.
 * 
 * Protocol flow:
 *   1. Listen on startup for ESC auto-announce (200ms window)
 *   2. If no announce, poll for ESC (0x40)
 *   3. Send broadcast handshake to finalize (0xFF)
 *   4. Send channel data at 50Hz with telemetry requests
 *   5. Parse telemetry responses
 * 
 * Half-duplex on PB6 (USART1), shared with PWM mode.
 */

#include "srxl2.h"
#include "uart.h"
#include "target.h"
#include <string.h>

/* Protocol constants */
#define SRXL2_ID                0xA6
#define SRXL2_HANDSHAKE_ID      0x21
#define SRXL2_CONTROL_ID        0xCD
#define SRXL2_TELEMETRY_ID      0x80
#define SRXL2_BIND_ID           0x41

/* Control commands */
#define CTRL_CMD_CHANNEL        0x00
#define CTRL_CMD_CHANNEL_FS     0x01

/* Device IDs */
#define DEVID_FLIGHT_CTRL       0x31    /* Us - flight controller (unit 1, not auto-announce) */
#define DEVID_ESC_BASE          0x40    /* ESC device type (unit 0 = auto-announce) */
#define DEVID_BROADCAST         0xFF

/* Timing */
#define STARTUP_LISTEN_MS       250     /* Listen for ESC auto-announce (spec says 200ms) */
#define HANDSHAKE_INTERVAL_MS   50
#define HANDSHAKE_TIMEOUT_MS    500     /* Time to wait for ESC response */
#define CHANNEL_INTERVAL_MS     20      /* 50Hz */
#define TELEMETRY_TIMEOUT_MS    500

/* Baud rates */
#define BAUD_115200             115200
#define BAUD_400000             400000
#define BAUD_SUPPORTED_MASK     0x01    /* Bit 0 = 400k supported */

/* Buffer sizes */
#define MAX_PACKET_SIZE         80
#define RX_BUFFER_SIZE          128

/* CRC-16 CCITT table */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

/* State machine */
typedef enum {
    STATE_IDLE,
    STATE_STARTUP_LISTEN,    /* Listen for ESC auto-announce */
    STATE_HANDSHAKE_ESC,     /* Poll for ESC if not found during listen */
    STATE_HANDSHAKE_FINAL,   /* Broadcast to finalize */
    STATE_RUNNING,           /* Normal operation - send channel data */
} srxl2_state_t;

/* Driver state */
static struct {
    srxl2_state_t state;
    uint32_t      baud_rate;
    uint32_t      state_start_ms;
    uint32_t      last_tx_ms;
    uint32_t      last_rx_ms;
    
    /* Discovered ESC */
    uint8_t       esc_device_id;
    bool          esc_found;
    uint8_t       esc_baud_supported;
    
    /* Channel data to send */
    uint16_t      throttle_us;      /* 1000-2000 */
    
    /* RX parsing */
    uint8_t       rx_buf[RX_BUFFER_SIZE];
    uint8_t       rx_pos;
    uint8_t       rx_expected_len;
    
    /* Telemetry */
    srxl2_telemetry_t telemetry;
    uint8_t       telem_request_counter;  /* Request telemetry every N frames */
    uint32_t      last_telem_rx_ms;       /* For re-handshake timeout */
    
    /* Stats */
    uint32_t      tx_count;
    uint32_t      rx_count;
    uint32_t      crc_errors;
    uint32_t      handshake_count;
    uint32_t      rehandshake_count;      /* Times we restarted handshake */
    
    /* Packet type counters for debugging */
    uint32_t      rx_handshake;
    uint32_t      rx_telemetry;
    uint32_t      rx_control;
    uint32_t      rx_other;
    uint8_t       last_pkt_type;
    uint32_t      last_handled_handshake;  /* For detecting late ESC power-on */
    
    bool          initialized;
} srxl2;

/* TX buffer */
static uint8_t tx_buf[MAX_PACKET_SIZE];

/* ============================================================================
 * CRC Calculation
 * ============================================================================ */

static uint16_t calc_crc(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t pos = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (uint16_t)((crc << 8) ^ crc16_table[pos]);
    }
    return crc;
}

/* ============================================================================
 * Packet Building
 * ============================================================================ */

static uint8_t build_handshake(uint8_t dest_id, uint8_t baud_field)
{
    /*
     * Handshake packet (14 bytes):
     *   0xA6 0x21 0x0E [src] [dest] [priority] [baud] [info] [uid:4] [crc:2]
     * 
     * Payload is little-endian, CRC is big-endian.
     * 
     * baud_field meaning depends on context:
     *   - When polling (dest = specific device): our supported baud rates
     *   - When broadcasting (dest = 0xFF): the agreed baud rate all devices must use
     */
    tx_buf[0] = SRXL2_ID;
    tx_buf[1] = SRXL2_HANDSHAKE_ID;
    tx_buf[2] = 14;  /* Total length */
    tx_buf[3] = DEVID_FLIGHT_CTRL;  /* Source = us */
    tx_buf[4] = dest_id;            /* Destination */
    tx_buf[5] = 10;                 /* Priority */
    tx_buf[6] = baud_field;         /* Baud rate field */
    tx_buf[7] = 0x00;               /* Info: no RF */
    
    /* UID - little-endian 32-bit value */
    tx_buf[8]  = 0x47;  /* 'G' - LSB */
    tx_buf[9]  = 0x46;  /* 'F' */
    tx_buf[10] = 0x4C;  /* 'L' */
    tx_buf[11] = 0x54;  /* 'T' - MSB */
    
    /* CRC over bytes 0-11, big-endian */
    uint16_t crc = calc_crc(tx_buf, 12);
    tx_buf[12] = (crc >> 8) & 0xFF;
    tx_buf[13] = crc & 0xFF;
    
    return 14;
}

static uint8_t build_channel_data(uint8_t reply_id, bool failsafe)
{
    /*
     * Control packet with channel data:
     *   0xA6 0xCD [len] [cmd] [replyID] [rssi] [losses:2] [mask:4] [values:2*N] [crc:2]
     * 
     * Channel value is 16-bit LITTLE-ENDIAN, center = 32768
     * Spektrum convention: 0 = full low, 32768 = center, 65535 = full high
     * 
     * Send channels 0-3 to cover all common throttle positions:
     *   - Surface: Ch1=steering, Ch2=throttle (indices 0,1)
     *   - Air: Ch1=ail, Ch2=ele, Ch3=thr, Ch4=rud (indices 0,1,2,3)
     */
    
    /* Convert 1000-2000us to Spektrum 16-bit range
     * Spec: "The lower 2 bits are reserved for future use and must be 0"
     */
    uint16_t throttle_value;
    if (srxl2.throttle_us <= 1000) {
        throttle_value = 0;
    } else if (srxl2.throttle_us >= 2000) {
        throttle_value = 0xFFFC;  /* Max with lower 2 bits clear */
    } else {
        throttle_value = ((uint32_t)(srxl2.throttle_us - 1000) * 65532) / 1000;
        throttle_value &= 0xFFFC;  /* Clear lower 2 bits */
    }
    
    /* Neutral value for non-throttle channels (1500µs = center) */
    uint16_t neutral = 0x8000;  /* 32768, lower bits already clear */
    
    /* Send 4 channels: mask = 0x0F (bits 0-3) 
     * Length: 3 (hdr) + 1 (cmd) + 1 (reply) + 1 (rssi) + 2 (losses) + 4 (mask) + 8 (4ch) + 2 (crc) = 22 */
    uint8_t len = 22;
    
    tx_buf[0] = SRXL2_ID;
    tx_buf[1] = SRXL2_CONTROL_ID;
    tx_buf[2] = len;
    tx_buf[3] = failsafe ? CTRL_CMD_CHANNEL_FS : CTRL_CMD_CHANNEL;
    tx_buf[4] = reply_id;   /* Request telemetry from this device */
    tx_buf[5] = 100;        /* RSSI: 100 = good signal */
    tx_buf[6] = 0;          /* Frame losses low byte */
    tx_buf[7] = 0;          /* Frame losses high byte */
    
    /* Channel mask - channels 0-3, little-endian 32-bit */
    tx_buf[8]  = 0x0F;      /* LSB: channels 0-3 */
    tx_buf[9]  = 0x00;
    tx_buf[10] = 0x00;
    tx_buf[11] = 0x00;      /* MSB */
    
    /* Channel 0 - throttle (surface ESC) - little-endian */
    tx_buf[12] = throttle_value & 0xFF;
    tx_buf[13] = (throttle_value >> 8) & 0xFF;

    /* Channels 1-3 - neutral */
    tx_buf[14] = neutral & 0xFF;
    tx_buf[15] = (neutral >> 8) & 0xFF;
    tx_buf[16] = neutral & 0xFF;
    tx_buf[17] = (neutral >> 8) & 0xFF;
    tx_buf[18] = neutral & 0xFF;
    tx_buf[19] = (neutral >> 8) & 0xFF;
    
    /* CRC - big-endian */
    uint16_t crc = calc_crc(tx_buf, len - 2);
    tx_buf[len - 2] = (crc >> 8) & 0xFF;
    tx_buf[len - 1] = crc & 0xFF;
    
    return len;
}

/* ============================================================================
 * Packet Parsing
 * ============================================================================ */

static bool parse_handshake(const uint8_t *pkt, uint8_t len)
{
    if (len < 14) return false;
    
    uint8_t src_id = pkt[3];
    uint8_t dest_id = pkt[4];
    uint8_t priority = pkt[5];
    uint8_t baud = pkt[6];
    uint8_t info = pkt[7];
    
    (void)priority;
    (void)info;
    
    /* Is this from an ESC? (device type 4 = upper nibble 0x4) */
    if ((src_id & 0xF0) == DEVID_ESC_BASE) {
        srxl2.esc_device_id = src_id;
        srxl2.esc_found = true;
        srxl2.esc_baud_supported = baud;
        srxl2.handshake_count++;
        return true;
    }
    
    /* Is this a broadcast response we should note? */
    if (dest_id == DEVID_BROADCAST) {
        /* Another device is finalizing handshake - note baud */
        srxl2.esc_baud_supported &= baud;
    }
    
    return true;
}

static bool parse_telemetry(const uint8_t *pkt, uint8_t len)
{
    /* Telemetry packet: Header(3) + destID(1) + payload(16) + crc(2) = 22 bytes */
    if (len < 22) return false;
    
    /* Telemetry payload starts at byte 4 */
    uint8_t sensor_id = pkt[4];
    uint8_t secondary_id = pkt[5];
    
    /* Reset re-handshake timer - we got telemetry! */
    srxl2.last_telem_rx_ms = target_millis();
    
    /*
     * ESC telemetry sensor IDs (from Spektrum telemetry spec):
     *   0x20 = ESC (RPM, volts, temp, current, etc.)
     * 
     * NOTE: Telemetry payload is X-Bus format which uses BIG-ENDIAN,
     * unlike SRXL2 native fields which are little-endian.
     */
    if (sensor_id == 0x20) {
        /* ESC telemetry frame format - bytes 6-19, BIG-ENDIAN */
        
        /* RPM - big-endian */
        srxl2.telemetry.rpm = (pkt[6] << 8) | pkt[7];
        
        /* Voltage in 0.01V units - big-endian */
        uint16_t volts_raw = (pkt[8] << 8) | pkt[9];
        srxl2.telemetry.voltage = volts_raw / 100.0f;
        
        /* Temperature in 0.1°C units - big-endian */
        uint16_t temp_raw = (pkt[10] << 8) | pkt[11];
        srxl2.telemetry.temperature = temp_raw / 10.0f;
        
        /* Current in 0.01A units - big-endian */
        uint16_t curr_raw = (pkt[12] << 8) | pkt[13];
        srxl2.telemetry.current = curr_raw / 100.0f;
        
        srxl2.telemetry.last_update_ms = target_millis();
        srxl2.telemetry.valid = true;
        
        return true;
    }
    
    (void)secondary_id;
    return false;
}

static void parse_packet(const uint8_t *pkt, uint8_t len)
{
    if (len < 5) return;
    if (pkt[0] != SRXL2_ID) return;
    
    /* Verify CRC - big-endian in packet */
    uint16_t rx_crc = (pkt[len - 2] << 8) | pkt[len - 1];
    uint16_t calc = calc_crc(pkt, len - 2);
    if (rx_crc != calc) {
        srxl2.crc_errors++;
        return;
    }
    
    srxl2.rx_count++;
    srxl2.last_rx_ms = target_millis();
    
    uint8_t pkt_type = pkt[1];
    srxl2.last_pkt_type = pkt_type;
    
    switch (pkt_type) {
        case SRXL2_HANDSHAKE_ID:
            srxl2.rx_handshake++;
            parse_handshake(pkt, len);
            break;
        case SRXL2_TELEMETRY_ID:
            srxl2.rx_telemetry++;
            parse_telemetry(pkt, len);
            break;
        case SRXL2_CONTROL_ID:
            srxl2.rx_control++;
            /* ESC echoing our control packets in half-duplex - ignore */
            break;
        default:
            srxl2.rx_other++;
            break;
    }
}

/* ============================================================================
 * RX Processing
 * ============================================================================ */

static void process_rx(void)
{
    int16_t byte;
    
    while ((byte = uart_read_byte(UART_ESC)) >= 0) {
        uint8_t data = (uint8_t)byte;
        
        if (srxl2.rx_pos == 0) {
            /* Looking for header */
            if (data == SRXL2_ID) {
                srxl2.rx_buf[0] = data;
                srxl2.rx_pos = 1;
            }
        } else if (srxl2.rx_pos == 1) {
            /* Packet type */
            srxl2.rx_buf[1] = data;
            srxl2.rx_pos = 2;
        } else if (srxl2.rx_pos == 2) {
            /* Length */
            srxl2.rx_buf[2] = data;
            srxl2.rx_expected_len = data;
            
            if (srxl2.rx_expected_len > MAX_PACKET_SIZE || srxl2.rx_expected_len < 5) {
                srxl2.rx_pos = 0;  /* Invalid length */
            } else {
                srxl2.rx_pos = 3;
            }
        } else {
            /* Payload + CRC */
            srxl2.rx_buf[srxl2.rx_pos++] = data;
            
            if (srxl2.rx_pos >= srxl2.rx_expected_len) {
                parse_packet(srxl2.rx_buf, srxl2.rx_expected_len);
                srxl2.rx_pos = 0;
            }
        }
    }
}

/* ============================================================================
 * State Machine
 * ============================================================================ */

static void send_packet(uint8_t len)
{
    uart_send(UART_ESC, tx_buf, len);
    srxl2.tx_count++;
    srxl2.last_tx_ms = target_millis();
}

static void run_state_machine(void)
{
    uint32_t now = target_millis();
    uint32_t elapsed = now - srxl2.state_start_ms;
    
    switch (srxl2.state) {
        case STATE_IDLE:
            break;
            
        case STATE_STARTUP_LISTEN:
            /*
             * Listen for ESC auto-announce. Per spec section 7.2.1:
             * ESC with unit ID 0 (default 0x40) will send handshake packets
             * every 50ms for 200ms after boot to prompt bus master.
             * 
             * We just listen - process_rx() will set esc_found if we
             * receive a valid handshake from an ESC.
             */
            if (srxl2.esc_found) {
                /* ESC announced itself - go straight to finalize */
                srxl2.state = STATE_HANDSHAKE_FINAL;
                srxl2.state_start_ms = now;
            } else if (elapsed >= STARTUP_LISTEN_MS) {
                /* No announcement heard - start polling */
                srxl2.state = STATE_HANDSHAKE_ESC;
                srxl2.state_start_ms = now;
            }
            break;
            
        case STATE_HANDSHAKE_ESC:
            /* Poll for ESC device - send our baud capabilities */
            if ((now - srxl2.last_tx_ms) >= HANDSHAKE_INTERVAL_MS) {
                uint8_t len = build_handshake(DEVID_ESC_BASE, BAUD_SUPPORTED_MASK);
                send_packet(len);
            }
            
            /* If ESC responded, move to finalize */
            if (srxl2.esc_found) {
                srxl2.state = STATE_HANDSHAKE_FINAL;
                srxl2.state_start_ms = now;
            }
            
            /* Timeout - keep trying (ESC might not be powered yet) */
            if (elapsed >= HANDSHAKE_TIMEOUT_MS) {
                srxl2.state_start_ms = now;
            }
            break;
            
        case STATE_HANDSHAKE_FINAL:
            /* Send broadcast handshake to finalize */
            if ((now - srxl2.last_tx_ms) >= 50) {
                /* Negotiate baud rate - use minimum supported by both parties */
                uint8_t agreed_baud = BAUD_SUPPORTED_MASK & srxl2.esc_baud_supported;
                
                /* Send broadcast with the agreed baud rate */
                uint8_t len = build_handshake(DEVID_BROADCAST, agreed_baud);
                send_packet(len);
                
                /* Switch our baud rate if 400k was agreed */
                if (agreed_baud & 0x01) {
                    target_delay_ms(2);  /* Let packet finish at current baud */
                    uart_deinit(UART_ESC);
                    uart_init(UART_ESC, BAUD_400000);
                    srxl2.baud_rate = BAUD_400000;
                }
                
                srxl2.state = STATE_RUNNING;
                srxl2.state_start_ms = now;
                srxl2.last_telem_rx_ms = now;  /* Reset telemetry timeout */
                srxl2.last_handled_handshake = srxl2.handshake_count;  /* Don't re-respond to initial handshakes */
            }
            break;
            
        case STATE_RUNNING:
            /* 
             * Handle late ESC power-on: if we see a new handshake FROM AN ESC
             * while running, respond with broadcast to complete the handshake.
             * Use handshake_count (ESC handshakes only), not rx_handshake (all).
             */
            if (srxl2.handshake_count > srxl2.last_handled_handshake) {
                srxl2.last_handled_handshake = srxl2.handshake_count;
                /* Send broadcast handshake to finalize */
                uint8_t agreed_baud = BAUD_SUPPORTED_MASK & srxl2.esc_baud_supported;
                uint8_t len = build_handshake(DEVID_BROADCAST, agreed_baud);
                send_packet(len);
            }
            
            /* Send channel data periodically */
            if ((now - srxl2.last_tx_ms) >= CHANNEL_INTERVAL_MS) {
                /* 
                 * Request telemetry every 10th frame (~5Hz) to not overwhelm ESC
                 * Some ESCs don't like being asked for telemetry every frame
                 */
                uint8_t reply_id = 0;  /* Default: no telemetry request */
                if (++srxl2.telem_request_counter >= 10) {
                    reply_id = srxl2.esc_device_id;
                    srxl2.telem_request_counter = 0;
                }
                
                uint8_t len = build_channel_data(reply_id, false);
                send_packet(len);
            }
            
            /* Telemetry validity timeout */
            if (srxl2.telemetry.valid && 
                (now - srxl2.telemetry.last_update_ms) > TELEMETRY_TIMEOUT_MS) {
                srxl2.telemetry.valid = false;
            }
            break;
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

bool srxl2_init(void)
{
    memset(&srxl2, 0, sizeof(srxl2));
    
    srxl2.throttle_us = 1500;  /* Neutral */
    srxl2.baud_rate = BAUD_115200;
    
    /* Initialize UART in half-duplex mode */
    if (!uart_init(UART_ESC, BAUD_115200)) {
        return false;
    }
    
    srxl2.state = STATE_STARTUP_LISTEN;
    srxl2.state_start_ms = target_millis();
    srxl2.last_tx_ms = srxl2.state_start_ms;
    srxl2.last_rx_ms = srxl2.state_start_ms;
    srxl2.last_telem_rx_ms = srxl2.state_start_ms;  /* Prevent immediate re-handshake */
    srxl2.initialized = true;
    
    return true;
}

void srxl2_deinit(void)
{
    if (!srxl2.initialized) return;
    
    uart_deinit(UART_ESC);
    srxl2.initialized = false;
    srxl2.state = STATE_IDLE;
}

void srxl2_process(void)
{
    if (!srxl2.initialized) return;
    
    process_rx();
    run_state_machine();
}

void srxl2_set_throttle(uint16_t throttle_us)
{
    if (throttle_us < 1000) throttle_us = 1000;
    if (throttle_us > 2000) throttle_us = 2000;
    srxl2.throttle_us = throttle_us;
}

void srxl2_set_throttle_crsf(uint16_t crsf_value)
{
    /* CRSF range: 172-1811 -> 1000-2000us */
    if (crsf_value < 172)  crsf_value = 172;
    if (crsf_value > 1811) crsf_value = 1811;
    
    uint16_t us = 1000 + ((uint32_t)(crsf_value - 172) * 1000) / 1639;
    srxl2_set_throttle(us);
}

bool srxl2_is_connected(void)
{
    return srxl2.esc_found && srxl2.state == STATE_RUNNING;
}

bool srxl2_telemetry_valid(void)
{
    return srxl2.telemetry.valid;
}

const srxl2_telemetry_t* srxl2_get_telemetry(void)
{
    return &srxl2.telemetry;
}

void srxl2_get_stats(uint32_t *tx, uint32_t *rx, uint32_t *crc_err, uint32_t *handshakes)
{
    if (tx) *tx = srxl2.tx_count;
    if (rx) *rx = srxl2.rx_count;
    if (crc_err) *crc_err = srxl2.crc_errors;
    if (handshakes) *handshakes = srxl2.handshake_count;
}

void srxl2_get_pkt_stats(uint32_t *hs, uint32_t *telem, uint32_t *ctrl, uint32_t *other, uint8_t *last_type)
{
    if (hs) *hs = srxl2.rx_handshake;
    if (telem) *telem = srxl2.rx_telemetry;
    if (ctrl) *ctrl = srxl2.rx_control;
    if (other) *other = srxl2.rx_other;
    if (last_type) *last_type = srxl2.last_pkt_type;
}

srxl2_state_info_t srxl2_get_state(void)
{
    srxl2_state_info_t info;
    info.state = srxl2.state;
    info.esc_found = srxl2.esc_found;
    info.esc_device_id = srxl2.esc_device_id;
    info.esc_baud_supported = srxl2.esc_baud_supported;
    info.baud_rate = srxl2.baud_rate;
    info.rehandshake_count = srxl2.rehandshake_count;
    return info;
}
