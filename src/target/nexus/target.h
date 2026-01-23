/**
 * GroundFlight Target: RadioMaster Nexus
 * 
 * Pin definitions and hardware configuration for the Nexus FBL controller.
 * MCU: STM32F722RET6
 * 
 * Note: Pin mappings derived from Betaflight target and board schematics.
 * Verify against actual hardware before use!
 */

#ifndef TARGET_NEXUS_H
#define TARGET_NEXUS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_HAL_STUBS
#include "stm32f7xx.h"
#else
#include "stm32f7xx_hal.h"
#endif

/* ============================================================================
 * System Configuration
 * ============================================================================ */

#define HSE_VALUE           8000000U    /* 8 MHz external crystal */
#define SYSTEM_CLOCK        216000000U  /* 216 MHz target clock */

/* ============================================================================
 * LED (for debug/status)
 * ============================================================================ */

/* Status LEDs - accent-active low */
#define LED1_GPIO_PORT      GPIOC
#define LED1_GPIO_PIN       14
#define LED2_GPIO_PORT      GPIOC
#define LED2_GPIO_PIN       15
#define LED_GPIO_CLK_EN     RCC_AHB1ENR_GPIOCEN

/* Aliases for primary LED */
#define LED_GPIO_PORT       LED1_GPIO_PORT
#define LED_GPIO_PIN        LED1_GPIO_PIN

/* ============================================================================
 * UART Assignments (from Rotorflight dump)
 * ============================================================================ */

/* UART4 - CRSF input from ELRS receiver */
#define CRSF_UART           4
#define CRSF_UART_BAUD      420000
#define CRSF_TX_GPIO_PORT   GPIOA
#define CRSF_TX_GPIO_PIN    1       /* PA1 - UART4_TX */
#define CRSF_RX_GPIO_PORT   GPIOA
#define CRSF_RX_GPIO_PIN    0       /* PA0 - UART4_RX */
#define CRSF_GPIO_AF        8       /* AF8 for UART4 */

/* UART3 - ESC telemetry (SRXL2, half-duplex) */
#define ESC_UART            3
#define ESC_UART_BAUD       115200  /* Default, can negotiate to 400000 */
#define ESC_TX_GPIO_PORT    GPIOB
#define ESC_TX_GPIO_PIN     11      /* PB11 - USART3_TX */
#define ESC_RX_GPIO_PORT    GPIOB
#define ESC_RX_GPIO_PIN     10      /* PB10 - USART3_RX */
#define ESC_GPIO_AF         7       /* AF7 for USART3 */

/* UART6 - Debug CLI */
#define CLI_UART            6
#define CLI_UART_BAUD       115200
#define CLI_TX_GPIO_PORT    GPIOC
#define CLI_TX_GPIO_PIN     7       /* PC7 - USART6_TX */
#define CLI_RX_GPIO_PORT    GPIOC
#define CLI_RX_GPIO_PIN     6       /* PC6 - USART6_RX */
#define CLI_GPIO_AF         8       /* AF8 for USART6 */

/* ============================================================================
 * SPI - IMU (ICM-42688-P)
 * ============================================================================ */

#define IMU_SPI             1
#define IMU_SPI_SPEED       24000000  /* 24 MHz max */

#define IMU_SCK_GPIO_PORT   GPIOA
#define IMU_SCK_GPIO_PIN    5       /* PA5 - SPI1_SCK */
#define IMU_MISO_GPIO_PORT  GPIOA
#define IMU_MISO_GPIO_PIN   6       /* PA6 - SPI1_MISO */
#define IMU_MOSI_GPIO_PORT  GPIOA
#define IMU_MOSI_GPIO_PIN   7       /* PA7 - SPI1_MOSI */
#define IMU_CS_GPIO_PORT    GPIOA
#define IMU_CS_GPIO_PIN     4       /* PA4 - SPI1_NSS (software CS) */
#define IMU_GPIO_AF         5       /* AF5 for SPI1 */

/* IMU interrupt pin */
#define IMU_INT_GPIO_PORT   GPIOA
#define IMU_INT_GPIO_PIN    15      /* PA15 - GYRO_EXTI */
#define IMU_INT_EXTI_LINE   15

/* ============================================================================
 * SPI - Flash (W25N01G, for blackbox - Phase 2)
 * ============================================================================ */

#define FLASH_SPI           2
#define FLASH_SPI_SPEED     104000000 /* 104 MHz max */

#define FLASH_SCK_GPIO_PORT  GPIOB
#define FLASH_SCK_GPIO_PIN   13      /* PB13 - SPI2_SCK */
#define FLASH_MISO_GPIO_PORT GPIOB
#define FLASH_MISO_GPIO_PIN  14      /* PB14 - SPI2_MISO */
#define FLASH_MOSI_GPIO_PORT GPIOB
#define FLASH_MOSI_GPIO_PIN  15      /* PB15 - SPI2_MOSI */
#define FLASH_CS_GPIO_PORT   GPIOB
#define FLASH_CS_GPIO_PIN    12      /* PB12 - FLASH_CS */
#define FLASH_GPIO_AF        5       /* AF5 for SPI2 */

/* ============================================================================
 * PWM Outputs (Servo/ESC)
 * ============================================================================ */

/* S1 - Steering servo */
#define SERVO_STEER_TIM         3
#define SERVO_STEER_TIM_CH      1
#define SERVO_STEER_GPIO_PORT   GPIOB
#define SERVO_STEER_GPIO_PIN    4       /* PB4 - TIM3_CH1 */
#define SERVO_STEER_GPIO_AF     2       /* AF2 for TIM3 */

/* S2 - Throttle/ESC (MOTOR 1 in Rotorflight, PWM fallback) */
#define SERVO_THROTTLE_TIM      4
#define SERVO_THROTTLE_TIM_CH   1
#define SERVO_THROTTLE_GPIO_PORT GPIOB
#define SERVO_THROTTLE_GPIO_PIN 6       /* PB6 - TIM4_CH1 */
#define SERVO_THROTTLE_GPIO_AF  2       /* AF2 for TIM4 */

/* S3 - E-brake servo */
#define SERVO_EBRAKE_TIM        3
#define SERVO_EBRAKE_TIM_CH     3
#define SERVO_EBRAKE_GPIO_PORT  GPIOB
#define SERVO_EBRAKE_GPIO_PIN   0       /* PB0 - TIM3_CH3 */
#define SERVO_EBRAKE_GPIO_AF    2       /* AF2 for TIM3 */

/* S4/TAIL - Aux output (SERVO 4 in Rotorflight) */
#define SERVO_AUX_TIM           2
#define SERVO_AUX_TIM_CH        2
#define SERVO_AUX_GPIO_PORT     GPIOB
#define SERVO_AUX_GPIO_PIN      3       /* PB3 - TIM2_CH2 */
#define SERVO_AUX_GPIO_AF       1       /* AF1 for TIM2 */

/* Default PWM frequency (Hz) */
#define SERVO_PWM_FREQ          50      /* Standard servo rate */

/* ============================================================================
 * Servo Channel Indices
 * ============================================================================ */

typedef enum {
    SERVO_CH_STEER = 0,
    SERVO_CH_THROTTLE,
    SERVO_CH_EBRAKE,
    SERVO_CH_AUX,
    SERVO_CH_COUNT
} servo_channel_t;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * Initialize target-specific hardware
 * Called early in startup, before main loop
 */
void target_init(void);

/**
 * Configure system clocks for 216 MHz operation
 * Called by SystemInit()
 */
void target_clock_init(void);

/**
 * LED control
 */
void target_led_on(void);
void target_led_off(void);
void target_led_toggle(void);

/**
 * Get system tick (milliseconds since boot)
 */
uint32_t target_millis(void);

/**
 * Delay in milliseconds
 */
void target_delay_ms(uint32_t ms);

/**
 * Reboot into DFU bootloader
 * Sets magic value in RAM and triggers reset
 */
void target_reboot_to_bootloader(void);

/**
 * Check for bootloader request (called from startup)
 * If magic value found in RAM, jumps directly to system bootloader
 */
void check_bootloader_request(void);

#endif /* TARGET_NEXUS_H */
