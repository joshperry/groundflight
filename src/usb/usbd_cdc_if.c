/**
 * USB CDC Interface
 * 
 * CDC class callbacks and RX buffer management
 */

#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

/* Receive buffer */
#define CDC_RX_BUF_SIZE    512
static uint8_t cdc_rx_buffer[CDC_RX_BUF_SIZE];

/* Circular buffer for received data */
#define CDC_CIRC_BUF_SIZE  1024
static uint8_t cdc_circ_buffer[CDC_CIRC_BUF_SIZE];
static volatile uint16_t cdc_circ_head = 0;
static volatile uint16_t cdc_circ_tail = 0;

/* Line coding (baud rate, etc) - not really used for virtual COM */
static USBD_CDC_LineCodingTypeDef line_coding = {
    .bitrate    = 115200,
    .format     = 0x00,  /* 1 stop bit */
    .paritytype = 0x00,  /* None */
    .datatype   = 0x08   /* 8 bits */
};

/* External reference to USB device handle */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Forward declarations */
static int8_t CDC_Init(void);
static int8_t CDC_DeInit(void);
static int8_t CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_CDC_Interface = {
    CDC_Init,
    CDC_DeInit,
    CDC_Control,
    CDC_Receive,
    CDC_TransmitCplt
};

static int8_t CDC_Init(void)
{
    /* Reset circular buffer */
    cdc_circ_head = 0;
    cdc_circ_tail = 0;
    
    /* Set receive buffer */
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buffer);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    
    return USBD_OK;
}

static int8_t CDC_DeInit(void)
{
    return USBD_OK;
}

static int8_t CDC_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;
    
    switch (cmd) {
        case CDC_SEND_ENCAPSULATED_COMMAND:
            break;

        case CDC_GET_ENCAPSULATED_RESPONSE:
            break;

        case CDC_SET_COMM_FEATURE:
            break;

        case CDC_GET_COMM_FEATURE:
            break;

        case CDC_CLEAR_COMM_FEATURE:
            break;

        case CDC_SET_LINE_CODING:
            line_coding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                             (pbuf[2] << 16) | (pbuf[3] << 24));
            line_coding.format = pbuf[4];
            line_coding.paritytype = pbuf[5];
            line_coding.datatype = pbuf[6];
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(line_coding.bitrate);
            pbuf[1] = (uint8_t)(line_coding.bitrate >> 8);
            pbuf[2] = (uint8_t)(line_coding.bitrate >> 16);
            pbuf[3] = (uint8_t)(line_coding.bitrate >> 24);
            pbuf[4] = line_coding.format;
            pbuf[5] = line_coding.paritytype;
            pbuf[6] = line_coding.datatype;
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            break;

        case CDC_SEND_BREAK:
            break;

        default:
            break;
    }

    return USBD_OK;
}

static int8_t CDC_Receive(uint8_t *pbuf, uint32_t *Len)
{
    /* Copy received data to circular buffer */
    for (uint32_t i = 0; i < *Len; i++) {
        uint16_t next_head = (cdc_circ_head + 1) % CDC_CIRC_BUF_SIZE;
        if (next_head != cdc_circ_tail) {
            cdc_circ_buffer[cdc_circ_head] = pbuf[i];
            cdc_circ_head = next_head;
        }
        /* else: buffer full, drop data */
    }
    
    /* Prepare for next reception */
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buffer);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    
    return USBD_OK;
}

static int8_t CDC_TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum)
{
    (void)pbuf;
    (void)Len;
    (void)epnum;
    return USBD_OK;
}

/* ============================================================================
 * Public API for reading received data
 * ============================================================================ */

uint16_t CDC_GetRxAvailable(void)
{
    if (cdc_circ_head >= cdc_circ_tail) {
        return cdc_circ_head - cdc_circ_tail;
    } else {
        return CDC_CIRC_BUF_SIZE - cdc_circ_tail + cdc_circ_head;
    }
}

int16_t CDC_GetRxByte(void)
{
    if (cdc_circ_head == cdc_circ_tail) {
        return -1;  /* Buffer empty */
    }
    
    uint8_t data = cdc_circ_buffer[cdc_circ_tail];
    cdc_circ_tail = (cdc_circ_tail + 1) % CDC_CIRC_BUF_SIZE;
    return data;
}

uint16_t CDC_GetRxData(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    
    while (count < max_len && cdc_circ_head != cdc_circ_tail) {
        buf[count++] = cdc_circ_buffer[cdc_circ_tail];
        cdc_circ_tail = (cdc_circ_tail + 1) % CDC_CIRC_BUF_SIZE;
    }
    
    return count;
}
