/* 
 * 
 * Interfacing with a MPU6050 gyroscope sensor for ESP32.
 */

#include "mpu6050.h"

static const char *TAG = "MPU6050";

esp_err_t i2c_config(void) {
    
    return ESP_OK;
}

esp_err_t mpu6050_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid arguments for reading register");
        return ESP_ERR_INVALID_ARG;
    }
    // temporary placeholder to avoid compiler warnings
    if (data != NULL) {
        memset(data, 0, len);
    }
    return ESP_OK;
}

esp_err_t mpu6050_write_register(uint8_t reg_addr, uint8_t data) {

    return ESP_OK;
}
                        
esp_err_t mpu6050_init(void) {
    esp_err_t ret = i2c_config();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C");
        return ret;
    }
    return ESP_OK;
}

esp_err_t mpu6050_read_data(mpu6050_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t raw_data[14];
    esp_err_t ret = mpu6050_read_register(MPU6050_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from MPU6050");
        return ret;
    }
    
    data->accelX = (raw_data[0] << 8) | raw_data[1];
    data->accelY = (raw_data[2] << 8) | raw_data[3];
    data->accelZ = (raw_data[4] << 8) | raw_data[5];
    data->gyroX = (raw_data[8] << 8) | raw_data[9]; 
    data->gyroY = (raw_data[10] << 8) | raw_data[11];
    data->gyroZ = (raw_data[12] << 8) | raw_data[13];

    return ESP_OK;
}

esp_err_t mpu6050_read_accel(int16_t *accelX, int16_t *accelY, int16_t *accelZ) {
    if (accelX == NULL || accelY == NULL || accelZ == NULL) {
        ESP_LOGE(TAG, "Accel data pointers are NULL");
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t raw_data[6];
    esp_err_t ret = mpu6050_read_register(MPU6050_ACCEL_XOUT_H, raw_data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data from MPU6050");
        return ret;
    }

    *accelX = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    *accelY = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    *accelZ = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    return ESP_OK;
}

esp_err_t mpu6050_read_gyro(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
    if (gyroX == NULL || gyroY == NULL || gyroZ == NULL) {
        ESP_LOGE(TAG, "Gyro data pointers are NULL");
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t raw_data[6];
    esp_err_t ret = mpu6050_read_register(MPU6050_GYRO_XOUT_H, raw_data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data from MPU6050");
        return ret;
    }

    *gyroX = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    *gyroY = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    *gyroZ = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    return ESP_OK;
}
