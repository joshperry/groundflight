/**
 * ICM-42688-P IMU Driver
 * 
 * High-performance 6-axis IMU for gyro stabilization.
 * We primarily use the gyro Z axis for yaw rate measurement.
 */

#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include <stdbool.h>

/* Gyro full-scale range options */
typedef enum {
    ICM42688_GYRO_FS_2000DPS = 0,  /* ±2000 °/s (default for RC) */
    ICM42688_GYRO_FS_1000DPS = 1,
    ICM42688_GYRO_FS_500DPS = 2,
    ICM42688_GYRO_FS_250DPS = 3,
    ICM42688_GYRO_FS_125DPS = 4,
    ICM42688_GYRO_FS_62_5DPS = 5,
    ICM42688_GYRO_FS_31_25DPS = 6,
    ICM42688_GYRO_FS_15_625DPS = 7,
} icm42688_gyro_fs_t;

/* Gyro output data rate options */
typedef enum {
    ICM42688_GYRO_ODR_32KHZ = 1,
    ICM42688_GYRO_ODR_16KHZ = 2,
    ICM42688_GYRO_ODR_8KHZ = 3,
    ICM42688_GYRO_ODR_4KHZ = 4,
    ICM42688_GYRO_ODR_2KHZ = 5,
    ICM42688_GYRO_ODR_1KHZ = 6,   /* Good balance for RC cars */
    ICM42688_GYRO_ODR_200HZ = 7,
    ICM42688_GYRO_ODR_100HZ = 8,
    ICM42688_GYRO_ODR_50HZ = 9,
    ICM42688_GYRO_ODR_25HZ = 10,
    ICM42688_GYRO_ODR_12_5HZ = 11,
} icm42688_gyro_odr_t;

/* Accel full-scale range options */
typedef enum {
    ICM42688_ACCEL_FS_16G = 0,
    ICM42688_ACCEL_FS_8G = 1,
    ICM42688_ACCEL_FS_4G = 2,
    ICM42688_ACCEL_FS_2G = 3,
} icm42688_accel_fs_t;

/* Raw sensor data */
typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
} icm42688_raw_data_t;

/* Scaled sensor data (in real units) */
typedef struct {
    float gyro_x;     /* deg/s */
    float gyro_y;     /* deg/s */
    float gyro_z;     /* deg/s */
    float accel_x;    /* g */
    float accel_y;    /* g */
    float accel_z;    /* g */
    float temp_c;     /* °C */
} icm42688_data_t;

/**
 * Initialize ICM-42688-P
 * @return true if device detected and configured
 */
bool icm42688_init(void);

/**
 * Check if device is detected
 */
bool icm42688_detect(void);

/**
 * Configure gyro settings
 */
bool icm42688_config_gyro(icm42688_gyro_fs_t fs, icm42688_gyro_odr_t odr);

/**
 * Configure accel settings
 */
bool icm42688_config_accel(icm42688_accel_fs_t fs, icm42688_gyro_odr_t odr);

/**
 * Check if new data is available
 */
bool icm42688_data_ready(void);

/**
 * Read raw sensor data
 */
bool icm42688_read_raw(icm42688_raw_data_t *data);

/**
 * Read and convert to real units
 */
bool icm42688_read(icm42688_data_t *data);

/**
 * Read just gyro Z (yaw rate) - optimized for main loop
 * @return Gyro Z in deg/s
 */
float icm42688_read_gyro_z(void);

/**
 * Get current gyro sensitivity (LSB per deg/s)
 */
float icm42688_get_gyro_scale(void);

/**
 * Read WHO_AM_I register (should return 0x47)
 */
uint8_t icm42688_who_am_i(void);

/**
 * Perform gyro calibration (measures bias at rest)
 * Device must be stationary during calibration!
 * @param samples Number of samples to average (e.g., 1000)
 */
void icm42688_calibrate_gyro(uint16_t samples);

/**
 * Get current gyro bias (from calibration)
 */
void icm42688_get_gyro_bias(float *bias_x, float *bias_y, float *bias_z);

/**
 * Set gyro bias directly (for external calibration loops)
 */
void icm42688_set_gyro_bias(float bias_x, float bias_y, float bias_z);

#endif /* ICM42688_H */
