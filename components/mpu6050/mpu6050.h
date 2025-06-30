/* 
 *
 * Header file for interfacing with the MPU6050. 
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include <math.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// MPU6050 definitions
#define MPU6050_ACCEL_XOUT_H 0x3B // accelerometer data registers (high byte)
#define MPU6050_GYRO_XOUT_H 0x43 // gyro data registers (high byte)
#define MPU6050_ADDRESS 0x68
#define MPU6050_PWR_MGMT_1 0x6B // power modes and clock source
#define MPU6050_WHO_AM_I 0x75

typedef struct {
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} mpu6050_data_t;


#endif // MPU6050_H