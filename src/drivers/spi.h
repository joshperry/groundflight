/**
 * SPI Driver
 * 
 * Low-level SPI interface for IMU and Flash
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>

/**
 * SPI bus identifiers
 */
typedef enum {
    SPI_BUS_IMU = 1,    /* SPI1 - ICM-42688 */
    SPI_BUS_FLASH = 2,  /* SPI2 - W25N01G */
} spi_bus_t;

/**
 * Initialize SPI bus
 */
bool spi_init(spi_bus_t bus);

/**
 * Transfer data (full duplex)
 * @param bus SPI bus
 * @param tx_data Data to transmit (can be NULL for read-only)
 * @param rx_data Buffer for received data (can be NULL for write-only)
 * @param len Number of bytes
 * @return true on success
 */
bool spi_transfer(spi_bus_t bus, const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/**
 * Write a single byte
 */
bool spi_write_byte(spi_bus_t bus, uint8_t data);

/**
 * Read a single byte
 */
uint8_t spi_read_byte(spi_bus_t bus);

/**
 * Assert chip select (active low)
 */
void spi_cs_low(spi_bus_t bus);

/**
 * Deassert chip select
 */
void spi_cs_high(spi_bus_t bus);

#endif /* SPI_H */
