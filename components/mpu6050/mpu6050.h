/* 
 *
 * Header file for interfacing with the MPU6050. 
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// I2C configuration
#define I2C_MASTER_SCL_IO 22 // GPIO number for SCL
#define I2C_MASTER_SDA_IO 21 // GPIO number for SDA

// MPU6050 definitions
#define MPU6050_ADDRESS 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B // accelerometer data registers (high byte)
#define MPU6050_GYRO_XOUT_H 0x43 // gyro data registers (high byte)
#define MPU6050_PWR_MGMT_1 0x6B // power modes and clock source
#define MPU6050_WHO_AM_I 0x75

#define CONFIG 0x1A
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG 0x1B

typedef struct {
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} mpu6050_data_t;

typedef struct {
    float accel_scale;
    float gyro_scale;
} mpu6050_scale_t;

typedef struct {
    int16_t accel_offset[3];
    int16_t gyro_offset[3];
} mpu6050_calibration_t;

typedef struct {
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
} mpu6050_scaled_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configures the I2C interface for communication with the MPU6050.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t i2c_config(void);

/**
 * @brief Writes data from the MPU6050 sensor.
 * @param data Pointer to a mpu6050_data_t structure to store the sensor data.
 * @param reg_addr Register address to write to.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_write_register(uint8_t reg_addr, uint8_t data);

/**
 * @brief Reads data from the MPU6050 sensor.
 * @param reg_addr Register address to read from.
 * @param data Pointer to buffer to store the read data.
 * @param len Number of bytes to read.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_read_register(uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Initializes the MPU6050 sensor.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Reads sensor data from the MPU6050.
 * @param data Pointer to a mpu6050_data_t structure to store the sensor data
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_read_data(mpu6050_data_t *data);

/**
 * @brief Reads accelerometer data from the MPU6050.
 * @param accelX Pointer to store the X-axis acceleration data.
 * @param accelY Pointer to store the Y-axis acceleration data.
 * @param accelZ Pointer to store the Z-axis acceleration data.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_read_accel(int16_t *accelX, int16_t *accelY, int16_t *accelZ);

/**
 * @brief Reads gyroscope data from the MPU6050.
 * @param gyroX Pointer to store the X-axis gyroscope data.
 * @param gyroY Pointer to store the Y-axis gyroscope data.
 * @param gyroZ Pointer to store the Z-axis gyroscope data.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_read_gyro(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);

/*
 * @brief Configures the MPU6050 sensor with specified accelerometer and gyroscope ranges.
 * @param accel_range Accelerometer range setting.
 * @param gyro_range Gyroscope range setting.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_configure(uint8_t accel_range, uint8_t gyro_range);

/*
 * @brief Calibrates the MPU6050 sensor to determine offsets for accelerometer and gyroscope.
 * @param offsets Pointer to mpu6050_calibration_t structure to store the calculated offsets.
 * @param samples Number of samples to use for calibration.
 * @return ESP_OK on success (1), or error code on failure (0). 
 */
esp_err_t mpu6050_calibrate(mpu6050_calibration_t *offsets, uint16_t samples);

/*
 * @brief Applies scale factors and offsets to raw sensor data to obtain scaled values.
 * @param data Pointer to mpu6050_scaled_data_t structure to store the scaled data.
 * @param offsets Pointer to mpu6050_calibration_t structure containing offsets.
 * @return ESP_OK on success (1), or error code on failure (0).
 */
esp_err_t mpu6050_get_scale_factors(mpu6050_scaled_data_t *data, const mpu6050_calibration_t *offsets);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H