/**
 * ICM-42688-P IMU Driver
 * 
 * High-performance 6-axis IMU for gyro stabilization.
 * SPI Mode 3 (CPOL=1, CPHA=1), max 24MHz
 */

#include "icm42688.h"
#include "spi.h"
#include "target.h"
#include <string.h>

/* ============================================================================
 * ICM-42688-P Register Map (Bank 0)
 * ============================================================================ */

#define ICM42688_REG_DEVICE_CONFIG      0x11
#define ICM42688_REG_DRIVE_CONFIG       0x13
#define ICM42688_REG_INT_CONFIG         0x14
#define ICM42688_REG_FIFO_CONFIG        0x16
#define ICM42688_REG_TEMP_DATA1         0x1D
#define ICM42688_REG_TEMP_DATA0         0x1E
#define ICM42688_REG_ACCEL_DATA_X1      0x1F
#define ICM42688_REG_ACCEL_DATA_X0      0x20
#define ICM42688_REG_ACCEL_DATA_Y1      0x21
#define ICM42688_REG_ACCEL_DATA_Y0      0x22
#define ICM42688_REG_ACCEL_DATA_Z1      0x23
#define ICM42688_REG_ACCEL_DATA_Z0      0x24
#define ICM42688_REG_GYRO_DATA_X1       0x25
#define ICM42688_REG_GYRO_DATA_X0       0x26
#define ICM42688_REG_GYRO_DATA_Y1       0x27
#define ICM42688_REG_GYRO_DATA_Y0       0x28
#define ICM42688_REG_GYRO_DATA_Z1       0x29
#define ICM42688_REG_GYRO_DATA_Z0       0x2A
#define ICM42688_REG_INT_STATUS         0x2D
#define ICM42688_REG_PWR_MGMT0          0x4E
#define ICM42688_REG_GYRO_CONFIG0       0x4F
#define ICM42688_REG_ACCEL_CONFIG0      0x50
#define ICM42688_REG_GYRO_CONFIG1       0x51
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM42688_REG_ACCEL_CONFIG1      0x53
#define ICM42688_REG_INT_CONFIG0        0x63
#define ICM42688_REG_INT_CONFIG1        0x64
#define ICM42688_REG_INT_SOURCE0        0x65
#define ICM42688_REG_WHO_AM_I           0x75
#define ICM42688_REG_REG_BANK_SEL       0x76

/* Device ID */
#define ICM42688_WHO_AM_I_VALUE         0x47

/* Power management bits */
#define ICM42688_PWR_GYRO_MODE_LN       (3 << 2)  /* Gyro low-noise mode */
#define ICM42688_PWR_ACCEL_MODE_LN      (3 << 0)  /* Accel low-noise mode */

/* Config bits */
#define ICM42688_DEVICE_CONFIG_SOFT_RESET   0x01

/* INT_STATUS bits */
#define ICM42688_INT_STATUS_DATA_RDY    0x08

/* SPI read flag */
#define ICM42688_SPI_READ               0x80

/* ============================================================================
 * State
 * ============================================================================ */

static bool initialized = false;
static icm42688_gyro_fs_t current_gyro_fs = ICM42688_GYRO_FS_2000DPS;
static icm42688_accel_fs_t current_accel_fs = ICM42688_ACCEL_FS_16G;

/* Gyro bias from calibration */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

/* Sensitivity lookup tables */
static const float gyro_sensitivity[] = {
    16.4f,    /* 2000 dps: 32768 / 2000 */
    32.8f,    /* 1000 dps */
    65.5f,    /* 500 dps */
    131.0f,   /* 250 dps */
    262.0f,   /* 125 dps */
    524.3f,   /* 62.5 dps */
    1048.6f,  /* 31.25 dps */
    2097.2f,  /* 15.625 dps */
};

static const float accel_sensitivity[] = {
    2048.0f,  /* 16g */
    4096.0f,  /* 8g */
    8192.0f,  /* 4g */
    16384.0f, /* 2g */
};

/* ============================================================================
 * Low-level SPI access
 * ============================================================================ */

static void icm42688_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value };
    
    spi_cs_low(SPI_BUS_IMU);
    spi_transfer(SPI_BUS_IMU, tx, NULL, 2);
    spi_cs_high(SPI_BUS_IMU);
}

static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { reg | ICM42688_SPI_READ, 0xFF };
    uint8_t rx[2];
    
    spi_cs_low(SPI_BUS_IMU);
    spi_transfer(SPI_BUS_IMU, tx, rx, 2);
    spi_cs_high(SPI_BUS_IMU);
    
    return rx[1];
}

static void icm42688_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx_buf[16];
    uint8_t rx_buf[16];
    
    tx_buf[0] = reg | ICM42688_SPI_READ;
    for (int i = 1; i <= len; i++) {
        tx_buf[i] = 0xFF;
    }
    
    spi_cs_low(SPI_BUS_IMU);
    spi_transfer(SPI_BUS_IMU, tx_buf, rx_buf, len + 1);
    spi_cs_high(SPI_BUS_IMU);
    
    memcpy(buf, &rx_buf[1], len);
}

/* ============================================================================
 * Public API
 * ============================================================================ */

uint8_t icm42688_who_am_i(void)
{
    return icm42688_read_reg(ICM42688_REG_WHO_AM_I);
}

bool icm42688_detect(void)
{
    /* Initialize SPI first */
    if (!spi_init(SPI_BUS_IMU)) {
        return false;
    }
    
    /* Small delay after power-up */
    target_delay_ms(10);
    
    /* Check WHO_AM_I */
    uint8_t who = icm42688_who_am_i();
    return (who == ICM42688_WHO_AM_I_VALUE);
}

bool icm42688_init(void)
{
    if (initialized) {
        return true;
    }
    
    /* Initialize SPI */
    if (!spi_init(SPI_BUS_IMU)) {
        return false;
    }
    
    /* Wait for device to be ready */
    target_delay_ms(10);
    
    /* Check device ID */
    if (icm42688_who_am_i() != ICM42688_WHO_AM_I_VALUE) {
        return false;
    }
    
    /* Soft reset */
    icm42688_write_reg(ICM42688_REG_DEVICE_CONFIG, ICM42688_DEVICE_CONFIG_SOFT_RESET);
    target_delay_ms(2);  /* Wait for reset */
    
    /* Wait for device to come back */
    target_delay_ms(10);
    
    /* Verify device is responding after reset */
    if (icm42688_who_am_i() != ICM42688_WHO_AM_I_VALUE) {
        return false;
    }
    
    /* Configure drive config for SPI speed */
    icm42688_write_reg(ICM42688_REG_DRIVE_CONFIG, 0x05);  /* Slew rate config */
    
    /* Configure gyro: ±2000 dps, 1kHz ODR */
    icm42688_config_gyro(ICM42688_GYRO_FS_2000DPS, ICM42688_GYRO_ODR_1KHZ);
    
    /* Configure accel: ±16g, 1kHz ODR */
    icm42688_config_accel(ICM42688_ACCEL_FS_16G, ICM42688_GYRO_ODR_1KHZ);
    
    /* Configure anti-alias filter */
    icm42688_write_reg(ICM42688_REG_GYRO_ACCEL_CONFIG0, 0x44);  /* 536Hz BW */
    
    /* Enable gyro and accel in low-noise mode */
    icm42688_write_reg(ICM42688_REG_PWR_MGMT0, 
                       ICM42688_PWR_GYRO_MODE_LN | ICM42688_PWR_ACCEL_MODE_LN);
    
    /* Wait for sensors to stabilize */
    target_delay_ms(50);
    
    initialized = true;
    return true;
}

bool icm42688_config_gyro(icm42688_gyro_fs_t fs, icm42688_gyro_odr_t odr)
{
    uint8_t config = ((uint8_t)fs << 5) | (uint8_t)odr;
    icm42688_write_reg(ICM42688_REG_GYRO_CONFIG0, config);
    current_gyro_fs = fs;
    return true;
}

bool icm42688_config_accel(icm42688_accel_fs_t fs, icm42688_gyro_odr_t odr)
{
    uint8_t config = ((uint8_t)fs << 5) | (uint8_t)odr;
    icm42688_write_reg(ICM42688_REG_ACCEL_CONFIG0, config);
    current_accel_fs = fs;
    return true;
}

bool icm42688_data_ready(void)
{
    uint8_t status = icm42688_read_reg(ICM42688_REG_INT_STATUS);
    return (status & ICM42688_INT_STATUS_DATA_RDY) != 0;
}

bool icm42688_read_raw(icm42688_raw_data_t *data)
{
    uint8_t buf[14];
    
    /* Read all sensor data in one burst (temp + accel + gyro) */
    icm42688_read_regs(ICM42688_REG_TEMP_DATA1, buf, 14);
    
    /* Parse data (big-endian) */
    data->temp    = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_x = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_y = (int16_t)((buf[4] << 8) | buf[5]);
    data->accel_z = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
    
    return true;
}

bool icm42688_read(icm42688_data_t *data)
{
    icm42688_raw_data_t raw;
    
    if (!icm42688_read_raw(&raw)) {
        return false;
    }
    
    float gyro_scale = gyro_sensitivity[current_gyro_fs];
    float accel_scale = accel_sensitivity[current_accel_fs];
    
    /* Convert to real units and apply calibration */
    data->gyro_x = (float)raw.gyro_x / gyro_scale - gyro_bias_x;
    data->gyro_y = (float)raw.gyro_y / gyro_scale - gyro_bias_y;
    data->gyro_z = (float)raw.gyro_z / gyro_scale - gyro_bias_z;
    
    data->accel_x = (float)raw.accel_x / accel_scale;
    data->accel_y = (float)raw.accel_y / accel_scale;
    data->accel_z = (float)raw.accel_z / accel_scale;
    
    /* Temperature: T_degC = (TEMP_DATA / 132.48) + 25 */
    data->temp_c = (float)raw.temp / 132.48f + 25.0f;
    
    return true;
}

float icm42688_read_gyro_z(void)
{
    uint8_t buf[2];
    
    /* Read just gyro Z registers for speed */
    icm42688_read_regs(ICM42688_REG_GYRO_DATA_Z1, buf, 2);
    
    int16_t raw_z = (int16_t)((buf[0] << 8) | buf[1]);
    float gyro_scale = gyro_sensitivity[current_gyro_fs];
    
    return (float)raw_z / gyro_scale - gyro_bias_z;
}

float icm42688_get_gyro_scale(void)
{
    return gyro_sensitivity[current_gyro_fs];
}

void icm42688_calibrate_gyro(uint16_t samples)
{
    if (samples == 0) samples = 1000;
    
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float gyro_scale = gyro_sensitivity[current_gyro_fs];
    
    for (uint16_t i = 0; i < samples; i++) {
        /* Wait for data ready */
        while (!icm42688_data_ready());
        
        uint8_t buf[6];
        icm42688_read_regs(ICM42688_REG_GYRO_DATA_X1, buf, 6);
        
        int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);
        
        sum_x += (float)raw_x / gyro_scale;
        sum_y += (float)raw_y / gyro_scale;
        sum_z += (float)raw_z / gyro_scale;
    }
    
    gyro_bias_x = sum_x / (float)samples;
    gyro_bias_y = sum_y / (float)samples;
    gyro_bias_z = sum_z / (float)samples;
}

void icm42688_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z)
{
    if (bias_x) *bias_x = gyro_bias_x;
    if (bias_y) *bias_y = gyro_bias_y;
    if (bias_z) *bias_z = gyro_bias_z;
}
