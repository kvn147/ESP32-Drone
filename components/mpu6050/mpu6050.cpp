/* 
 * 
 * Interfacing with a MPU6050 gyroscope sensor for ESP32.
 */

#include "mpu6050.h"

static const char *TAG = "MPU6050";

// I2C configuration constants
#define I2C_MASTER_PORT_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0

esp_err_t i2c_config(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    esp_err_t ret = i2c_param_config((i2c_port_t)I2C_MASTER_PORT_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters");
        return ret;
    }
    
    ret = i2c_driver_install((i2c_port_t)I2C_MASTER_PORT_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C configured successfully");
    return ESP_OK;
}

esp_err_t mpu6050_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid arguments for reading register");
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Start condition
    i2c_master_start(cmd);
    
    // Write device address with write bit
    i2c_master_write_byte(cmd, (MPU6050_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    
    // Write register address
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    
    // Repeated start condition
    i2c_master_start(cmd);
    
    // Write device address with read bit
    i2c_master_write_byte(cmd, (MPU6050_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    
    // Read data
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    
    // Stop condition
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)I2C_MASTER_PORT_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X", reg_addr);
    }
    
    return ret;
}

esp_err_t mpu6050_write_register(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Start condition
    i2c_master_start(cmd);
    
    // Write device address with write bit
    i2c_master_write_byte(cmd, (MPU6050_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    
    // Write register address
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    
    // Write data
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    
    // Stop condition
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)I2C_MASTER_PORT_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X", reg_addr);
    }
    
    return ret;
}
                        
esp_err_t mpu6050_init(void) {
    esp_err_t ret = i2c_config();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C");
        return ret;
    }
    
    // Wake up the MPU6050 (clear sleep bit)
    ret = mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }
    
    // Give the device some time to wake up
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Verify communication by reading WHO_AM_I register
    uint8_t who_am_i;
    ret = mpu6050_read_register(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", who_am_i);
    
    // The WHO_AM_I register should contain 0x68
    if (who_am_i != 0x68) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I response: expected 0x68, got 0x%02X", who_am_i);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
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

esp_err_t mpu6050_configure(uint8_t accel_range, uint8_t gyro_range) {
    esp_err_t ret = mpu6050_write_register(MPU6050_ACCEL_CONFIG, accel_range);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer range");
        return ret;
    }
    
    ret = mpu6050_write_register(MPU6050_GYRO_CONFIG, gyro_range);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope range");
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 configured successfully");
    return ESP_OK;
}

esp_err_t mpu6050_calibrate(mpu6050_calibration_t *offsets, uint16_t samples) {
    if (offsets == NULL || samples == 0) {
        ESP_LOGE(TAG, "Invalid arguments for calibration");
        return ESP_ERR_INVALID_ARG;
    }

    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};

    mpu6050_data_t data;
    for (uint16_t i = 0; i < samples; i++) {
        esp_err_t ret = mpu6050_read_data(&data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read data during calibration");
            return ret;
        }

        accel_sum[0] += data.accelX;
        accel_sum[1] += data.accelY;
        accel_sum[2] += data.accelZ;

        gyro_sum[0] += data.gyroX;
        gyro_sum[1] += data.gyroY;
        gyro_sum[2] += data.gyroZ;

        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay between samples
    }

    offsets->accel_offset[0] = (int16_t)(accel_sum[0] / samples);
    offsets->accel_offset[1] = (int16_t)(accel_sum[1] / samples);
    offsets->accel_offset[2] = (int16_t)(accel_sum[2] / samples);

    offsets->gyro_offset[0] = (int16_t)(gyro_sum[0] / samples);
    offsets->gyro_offset[1] = (int16_t)(gyro_sum[1] / samples);
    offsets->gyro_offset[2] = (int16_t)(gyro_sum[2] / samples);

    ESP_LOGI(TAG, "MPU6050 calibration completed");
    return ESP_OK;
}

esp_err_t mpu6050_get_scale_factors(mpu6050_scaled_data_t *data, const mpu6050_calibration_t *offsets) {
    if (data == NULL || offsets == NULL) {
        ESP_LOGE(TAG, "Invalid arguments for getting scale factors");
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050_data_t raw_data;
    esp_err_t ret = mpu6050_read_data(&raw_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data for scaling");
        return ret;
    }

    // Apply offsets
    int16_t corrected_accelX = raw_data.accelX - offsets->accel_offset[0];
    int16_t corrected_accelY = raw_data.accelY - offsets->accel_offset[1];
    int16_t corrected_accelZ = raw_data.accelZ - offsets->accel_offset[2];

    int16_t corrected_gyroX = raw_data.gyroX - offsets->gyro_offset[0];
    int16_t corrected_gyroY = raw_data.gyroY - offsets->gyro_offset[1];
    int16_t corrected_gyroZ = raw_data.gyroZ - offsets->gyro_offset[2];

    // Convert to scaled values (assuming default sensitivity settings)
    data->accel_x = (float)corrected_accelX / 16384.0f; // assuming ±2g
    data->accel_y = (float)corrected_accelY / 16384.0f;
    data->accel_z = (float)corrected_accelZ / 16384.0f;

    data->gyro_x = (float)corrected_gyroX / 131.0f; // assuming ±250°/s
    data->gyro_y = (float)corrected_gyroY / 131.0f;
    data->gyro_z = (float)corrected_gyroZ / 131.0f;

    return ESP_OK;
}
