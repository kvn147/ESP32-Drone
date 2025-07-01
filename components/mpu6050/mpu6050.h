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

typedef struct {
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} mpu6050_data_t;

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

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H