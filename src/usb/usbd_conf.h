/**
 * USB Device Configuration
 * 
 * Board-specific USB device settings for STM32F722
 */

#ifndef USBD_CONF_H
#define USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USB Device definitions */
#define USBD_MAX_NUM_INTERFACES     1
#define USBD_MAX_NUM_CONFIGURATION  1
#define USBD_MAX_STR_DESC_SIZ       512
#define USBD_SUPPORT_USER_STRING    0
#define USBD_DEBUG_LEVEL            0
#define USBD_SELF_POWERED           0

/* CDC specific */
#define USBD_CDC_INTERVAL           2000  /* Polling interval in frames (2ms) */

/* Memory management macros */
#define USBD_malloc         malloc
#define USBD_free           free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

/* Debug macros */
#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)    printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)
#define USBD_ErrLog(...)    printf("ERROR: ");\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)    printf("DEBUG: ");\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* USBD_CONF_H */
