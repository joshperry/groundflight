/**
 * USB CDC Wrapper
 * 
 * High-level USB CDC API for GroundFlight
 */

#include "usb_cdc.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* USB Device handle */
USBD_HandleTypeDef hUsbDeviceFS;

/* Static TX buffer - USB DMA reads from this after usb_cdc_send() returns */
#define CDC_TX_BUF_SIZE 256
static uint8_t tx_buf[CDC_TX_BUF_SIZE];

/* External functions from usbd_cdc_if.c */
extern uint16_t CDC_GetRxAvailable(void);
extern int16_t CDC_GetRxByte(void);
extern uint16_t CDC_GetRxData(uint8_t *buf, uint16_t max_len);

void usb_cdc_init(void)
{
    /* Initialize USB device library */
    USBD_Init(&hUsbDeviceFS, &FS_Desc, 0);
    
    /* Register CDC class */
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    
    /* Register CDC interface callbacks */
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_Interface);
    
    /* Start USB device */
    USBD_Start(&hUsbDeviceFS);
}

void usb_cdc_process(void)
{
    /* USB events are handled in interrupt context via OTG_FS_IRQHandler */
    /* Nothing to poll here */
}

bool usb_cdc_is_connected(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

void usb_cdc_send(const uint8_t *data, uint16_t len)
{
    if (!usb_cdc_is_connected() || len == 0) {
        return;
    }
    
    /* Wait for previous transmission to complete */
    uint32_t timeout = 100000;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    while (hcdc->TxState != 0 && timeout > 0) {
        timeout--;
    }
    
    if (timeout == 0) {
        return;  /* Timeout, drop data */
    }

    /* Copy into static buffer so DMA has a stable source after we return */
    if (len > CDC_TX_BUF_SIZE) {
        len = CDC_TX_BUF_SIZE;
    }
    memcpy(tx_buf, data, len);

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_buf, len);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

void usb_cdc_print(const char *str)
{
    usb_cdc_send((const uint8_t *)str, strlen(str));
}

uint16_t usb_cdc_available(void)
{
    return CDC_GetRxAvailable();
}

uint16_t usb_cdc_read(uint8_t *buf, uint16_t max_len)
{
    return CDC_GetRxData(buf, max_len);
}

int16_t usb_cdc_read_byte(void)
{
    return CDC_GetRxByte();
}
