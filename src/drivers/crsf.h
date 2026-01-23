/**
 * CRSF (Crossfire) Protocol Parser
 * 
 * Parses ELRS/CRSF receiver data from UART4
 * 420000 baud, standard UART (non-inverted for internal ELRS RX)
 * 
 * Frame format:
 *   [sync 0xC8] [len] [type] [payload...] [crc8]
 * 
 * Protocol spec: https://github.com/crsf-wg/crsf/wiki
 */

#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol constants (matching xbox-elrs transmitter) */
#define CRSF_SYNC_BYTE           0xC8
#define CRSF_BAUDRATE            420000

/* Channel value range (11-bit) */
#define CRSF_CHANNEL_MIN         172    /* 988us equivalent */
#define CRSF_CHANNEL_MID         992    /* 1500us equivalent */
#define CRSF_CHANNEL_MAX         1811   /* 2012us equivalent */

/* Number of RC channels */
#define CRSF_NUM_CHANNELS        16

/* Frame types */
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED   0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS      0x14
#define CRSF_FRAMETYPE_BATTERY_SENSOR       0x08
#define CRSF_FRAMETYPE_GPS                  0x02

/* Frame size limits */
#define CRSF_FRAME_SIZE_MAX      64
#define CRSF_PAYLOAD_SIZE_MAX    60

/* RC channels frame: sync(1) + len(1) + type(1) + payload(22) + crc(1) = 26 */
#define CRSF_RC_CHANNELS_FRAME_SIZE  26

/* Link statistics data */
typedef struct {
    uint8_t  uplink_rssi_1;      /* RSSI antenna 1 (dBm * -1) */
    uint8_t  uplink_rssi_2;      /* RSSI antenna 2 (dBm * -1) */
    uint8_t  uplink_link_quality; /* 0-100% */
    int8_t   uplink_snr;         /* SNR in dB */
    uint8_t  active_antenna;     /* 0 or 1 */
    uint8_t  rf_mode;            /* ELRS rate index */
    uint8_t  uplink_tx_power;    /* TX power index */
    uint8_t  downlink_rssi;      /* Downlink RSSI (dBm * -1) */
    uint8_t  downlink_link_quality;
    int8_t   downlink_snr;
} crsf_link_stats_t;

/* Parsed CRSF data */
typedef struct {
    uint16_t channels[CRSF_NUM_CHANNELS];  /* Channel values (172-1811) */
    crsf_link_stats_t link;                /* Link statistics */
    
    uint32_t last_rc_frame_ms;             /* Timestamp of last RC frame */
    uint32_t last_link_frame_ms;           /* Timestamp of last link stats */
    uint32_t frame_count;                  /* Total frames received */
    uint32_t error_count;                  /* CRC/framing errors */
    
    bool     failsafe;                     /* True if no recent valid frames */
} crsf_state_t;

/**
 * Initialize CRSF parser and UART
 * 
 * @return true on success
 */
bool crsf_init(void);

/**
 * Process incoming CRSF data
 * 
 * Call this frequently from main loop to parse buffered UART data.
 * Non-blocking - returns immediately if no complete frame available.
 */
void crsf_process(void);

/**
 * Get current CRSF state
 * 
 * @return Pointer to current state (read-only)
 */
const crsf_state_t* crsf_get_state(void);

/**
 * Check if CRSF receiver is connected and sending data
 * 
 * @return true if valid frames received recently
 */
bool crsf_is_connected(void);

/**
 * Convert CRSF channel value to normalized float
 * 
 * @param value CRSF channel value (172-1811)
 * @return Normalized value (-1.0 to 1.0)
 */
static inline float crsf_to_float(uint16_t value)
{
    /* Map 172-1811 to -1.0 to 1.0 */
    float normalized = ((float)value - CRSF_CHANNEL_MID) / 
                       (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID);
    
    /* Clamp to range */
    if (normalized < -1.0f) normalized = -1.0f;
    if (normalized >  1.0f) normalized =  1.0f;
    
    return normalized;
}

/**
 * Convert CRSF channel value to microseconds (servo pulse width)
 * 
 * @param value CRSF channel value (172-1811)
 * @return Pulse width in microseconds (~988-2012)
 */
static inline uint16_t crsf_to_us(uint16_t value)
{
    /* Linear map: 172 -> 988us, 1811 -> 2012us */
    /* Formula: us = 988 + (value - 172) * (2012 - 988) / (1811 - 172) */
    /*            = 988 + (value - 172) * 1024 / 1639 */
    return 988 + ((uint32_t)(value - CRSF_CHANNEL_MIN) * 1024) / 
                 (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
}

#endif /* CRSF_H */
