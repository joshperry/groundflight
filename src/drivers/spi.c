/**
 * SPI Driver
 * 
 * Low-level SPI interface for IMU and Flash
 */

#include "spi.h"
#include "target.h"
#include "stm32f7xx_hal.h"

/* SPI handles */
static SPI_HandleTypeDef hspi1;  /* IMU */
static SPI_HandleTypeDef hspi2;  /* Flash */

/* Initialization flags */
static bool spi1_initialized = false;
static bool spi2_initialized = false;

/**
 * Initialize SPI1 for IMU (ICM-42688-P)
 * 
 * Pins (from Rotorflight dump):
 *   PA4 - CS (software controlled)
 *   PA5 - SCK
 *   PA6 - MISO
 *   PA7 - MOSI
 * 
 * ICM-42688-P max SPI clock: 24 MHz
 * We'll run at ~13.5 MHz (APB2=108MHz / 8)
 */
static bool spi1_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable clocks */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* Configure CS pin as GPIO output (manual control) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* CS high (deselected) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    
    /* Configure SPI pins */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Configure SPI1 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;   /* CPOL=1 for ICM-42688 */
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;        /* CPHA=1 for ICM-42688 */
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  /* 108MHz/8 = 13.5MHz */
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        return false;
    }
    
    /* Enable SPI */
    __HAL_SPI_ENABLE(&hspi1);
    
    spi1_initialized = true;
    return true;
}

/**
 * Initialize SPI2 for Flash (W25N01G)
 * 
 * Pins:
 *   PB12 - CS
 *   PB13 - SCK
 *   PB14 - MISO
 *   PB15 - MOSI
 */
static bool spi2_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable clocks */
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure CS pin as GPIO output */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* CS high (deselected) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    
    /* Configure SPI pins */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Configure SPI2 */
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;  /* 54MHz/4 = 13.5MHz */
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        return false;
    }
    
    __HAL_SPI_ENABLE(&hspi2);
    
    spi2_initialized = true;
    return true;
}

bool spi_init(spi_bus_t bus)
{
    switch (bus) {
        case SPI_BUS_IMU:
            return spi1_init();
        case SPI_BUS_FLASH:
            return spi2_init();
        default:
            return false;
    }
}

static SPI_HandleTypeDef *get_spi_handle(spi_bus_t bus)
{
    switch (bus) {
        case SPI_BUS_IMU:
            return &hspi1;
        case SPI_BUS_FLASH:
            return &hspi2;
        default:
            return NULL;
    }
}

bool spi_transfer(spi_bus_t bus, const uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    SPI_HandleTypeDef *hspi = get_spi_handle(bus);
    if (!hspi) return false;
    
    HAL_StatusTypeDef status;
    
    if (tx_data && rx_data) {
        status = HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx_data, rx_data, len, 100);
    } else if (tx_data) {
        status = HAL_SPI_Transmit(hspi, (uint8_t *)tx_data, len, 100);
    } else if (rx_data) {
        status = HAL_SPI_Receive(hspi, rx_data, len, 100);
    } else {
        return false;
    }
    
    return (status == HAL_OK);
}

bool spi_write_byte(spi_bus_t bus, uint8_t data)
{
    return spi_transfer(bus, &data, NULL, 1);
}

uint8_t spi_read_byte(spi_bus_t bus)
{
    uint8_t data = 0;
    uint8_t dummy = 0xFF;
    spi_transfer(bus, &dummy, &data, 1);
    return data;
}

void spi_cs_low(spi_bus_t bus)
{
    switch (bus) {
        case SPI_BUS_IMU:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
            break;
        case SPI_BUS_FLASH:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            break;
    }
}

void spi_cs_high(spi_bus_t bus)
{
    switch (bus) {
        case SPI_BUS_IMU:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
            break;
        case SPI_BUS_FLASH:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            break;
    }
}
