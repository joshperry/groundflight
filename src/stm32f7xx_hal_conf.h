/**
 * STM32F7xx HAL Configuration
 * 
 * Project-specific configuration for STM32CubeF7 HAL.
 * Only enable modules we actually use to minimize code size.
 */

#ifndef STM32F7XX_HAL_CONF_H
#define STM32F7XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Module Selection
 * ============================================================================ */

#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* Disabled modules (uncomment to enable)
#define HAL_ADC_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
#define HAL_CEC_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED
#define HAL_CRYP_MODULE_ENABLED
#define HAL_DAC_MODULE_ENABLED
#define HAL_DCMI_MODULE_ENABLED
#define HAL_DMA2D_MODULE_ENABLED
#define HAL_ETH_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_HASH_MODULE_ENABLED
#define HAL_HCD_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_I2S_MODULE_ENABLED
#define HAL_IRDA_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_LPTIM_MODULE_ENABLED
#define HAL_LTDC_MODULE_ENABLED
#define HAL_MDIOS_MODULE_ENABLED
#define HAL_MMC_MODULE_ENABLED
#define HAL_NAND_MODULE_ENABLED
#define HAL_NOR_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_QSPI_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SAI_MODULE_ENABLED
#define HAL_SD_MODULE_ENABLED
#define HAL_SDRAM_MODULE_ENABLED
#define HAL_SMARTCARD_MODULE_ENABLED
#define HAL_SMBUS_MODULE_ENABLED
#define HAL_SPDIFRX_MODULE_ENABLED
#define HAL_SRAM_MODULE_ENABLED
#define HAL_USART_MODULE_ENABLED
#define HAL_WWDG_MODULE_ENABLED
*/

/* ============================================================================
 * Oscillator Values
 * ============================================================================ */

#if !defined(HSE_VALUE)
#define HSE_VALUE    8000000U  /* 8 MHz external crystal on Nexus */
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    100U
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE    16000000U  /* 16 MHz internal oscillator */
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE    32000U
#endif

#if !defined(LSE_VALUE)
#define LSE_VALUE    32768U
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    5000U
#endif

#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE    12288000U
#endif

/* ============================================================================
 * System Configuration
 * ============================================================================ */

#define VDD_VALUE                  3300U  /* 3.3V */
#define TICK_INT_PRIORITY          0x0FU
#define USE_RTOS                   0U
#define PREFETCH_ENABLE            1U
#define ART_ACCELERATOR_ENABLE     1U
#define INSTRUCTION_CACHE_ENABLE   1U
#define DATA_CACHE_ENABLE          1U

/* ============================================================================
 * Ethernet Configuration (disabled, but required by HAL)
 * ============================================================================ */

#define ETH_TX_DESC_CNT            4U
#define ETH_RX_DESC_CNT            4U
#define ETH_MAC_ADDR0              0x02U
#define ETH_MAC_ADDR1              0x00U
#define ETH_MAC_ADDR2              0x00U
#define ETH_MAC_ADDR3              0x00U
#define ETH_MAC_ADDR4              0x00U
#define ETH_MAC_ADDR5              0x00U

/* ============================================================================
 * SPI Configuration
 * ============================================================================ */

#define USE_SPI_CRC                0U

/* ============================================================================
 * HAL Module Includes
 * ============================================================================ */

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f7xx_hal_rcc.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32f7xx_hal_exti.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f7xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f7xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f7xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32f7xx_hal_adc.h"
#endif

#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f7xx_hal_can.h"
#endif

#ifdef HAL_CEC_MODULE_ENABLED
#include "stm32f7xx_hal_cec.h"
#endif

#ifdef HAL_CRC_MODULE_ENABLED
#include "stm32f7xx_hal_crc.h"
#endif

#ifdef HAL_CRYP_MODULE_ENABLED
#include "stm32f7xx_hal_cryp.h"
#endif

#ifdef HAL_DAC_MODULE_ENABLED
#include "stm32f7xx_hal_dac.h"
#endif

#ifdef HAL_DCMI_MODULE_ENABLED
#include "stm32f7xx_hal_dcmi.h"
#endif

#ifdef HAL_DMA2D_MODULE_ENABLED
#include "stm32f7xx_hal_dma2d.h"
#endif

#ifdef HAL_ETH_MODULE_ENABLED
#include "stm32f7xx_hal_eth.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f7xx_hal_flash.h"
#endif

#ifdef HAL_HASH_MODULE_ENABLED
#include "stm32f7xx_hal_hash.h"
#endif

#ifdef HAL_HCD_MODULE_ENABLED
#include "stm32f7xx_hal_hcd.h"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32f7xx_hal_i2c.h"
#endif

#ifdef HAL_I2S_MODULE_ENABLED
#include "stm32f7xx_hal_i2s.h"
#endif

#ifdef HAL_IRDA_MODULE_ENABLED
#include "stm32f7xx_hal_irda.h"
#endif

#ifdef HAL_IWDG_MODULE_ENABLED
#include "stm32f7xx_hal_iwdg.h"
#endif

#ifdef HAL_LPTIM_MODULE_ENABLED
#include "stm32f7xx_hal_lptim.h"
#endif

#ifdef HAL_LTDC_MODULE_ENABLED
#include "stm32f7xx_hal_ltdc.h"
#endif

#ifdef HAL_MDIOS_MODULE_ENABLED
#include "stm32f7xx_hal_mdios.h"
#endif

#ifdef HAL_MMC_MODULE_ENABLED
#include "stm32f7xx_hal_mmc.h"
#endif

#ifdef HAL_NAND_MODULE_ENABLED
#include "stm32f7xx_hal_nand.h"
#endif

#ifdef HAL_NOR_MODULE_ENABLED
#include "stm32f7xx_hal_nor.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f7xx_hal_pcd.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
#include "stm32f7xx_hal_pcd.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f7xx_hal_pwr.h"
#endif

#ifdef HAL_QSPI_MODULE_ENABLED
#include "stm32f7xx_hal_qspi.h"
#endif

#ifdef HAL_RNG_MODULE_ENABLED
#include "stm32f7xx_hal_rng.h"
#endif

#ifdef HAL_RTC_MODULE_ENABLED
#include "stm32f7xx_hal_rtc.h"
#endif

#ifdef HAL_SAI_MODULE_ENABLED
#include "stm32f7xx_hal_sai.h"
#endif

#ifdef HAL_SD_MODULE_ENABLED
#include "stm32f7xx_hal_sd.h"
#endif

#ifdef HAL_SDRAM_MODULE_ENABLED
#include "stm32f7xx_hal_sdram.h"
#endif

#ifdef HAL_SMARTCARD_MODULE_ENABLED
#include "stm32f7xx_hal_smartcard.h"
#endif

#ifdef HAL_SMBUS_MODULE_ENABLED
#include "stm32f7xx_hal_smbus.h"
#endif

#ifdef HAL_SPDIFRX_MODULE_ENABLED
#include "stm32f7xx_hal_spdifrx.h"
#endif

#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32f7xx_hal_spi.h"
#endif

#ifdef HAL_SRAM_MODULE_ENABLED
#include "stm32f7xx_hal_sram.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f7xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f7xx_hal_uart.h"
#endif

#ifdef HAL_USART_MODULE_ENABLED
#include "stm32f7xx_hal_usart.h"
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
#include "stm32f7xx_hal_wwdg.h"
#endif

/* ============================================================================
 * Assert Configuration
 * ============================================================================ */

/* #define USE_FULL_ASSERT 1U */

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32F7XX_HAL_CONF_H */
