/*
 * Unit tests for MPU6050 driver - calibration and scaling functions
 */

#include "unity.h"
#include "mpu6050.h"
#include <string.h>

// Mock data for testing
static mpu6050_data_t mock_sensor_data;
static esp_err_t mock_read_return_value = ESP_OK;
static bool mock_read_data_called = false;

// Override mpu6050_read_data for testing
esp_err_t __real_mpu6050_read_data(mpu6050_data_t *data);
esp_err_t __wrap_mpu6050_read_data(mpu6050_data_t *data) {
    mock_read_data_called = true;
    if (mock_read_return_value != ESP_OK) {
        return mock_read_return_value;
    }
    if (data != NULL) {
        memcpy(data, &mock_sensor_data, sizeof(mpu6050_data_t));
    }
    return ESP_OK;
}

void setUp(void) {
    // Reset mock data before each test
    memset(&mock_sensor_data, 0, sizeof(mpu6050_data_t));
    mock_read_return_value = ESP_OK;
    mock_read_data_called = false;
}

void tearDown(void) {
    // Cleanup after each test
}

// ========== mpu6050_calibrate Tests ==========

void test_mpu6050_calibrate_null_offsets(void) {
    esp_err_t ret = mpu6050_calibrate(NULL, 100);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

void test_mpu6050_calibrate_zero_samples(void) {
    mpu6050_calibration_t offsets;
    esp_err_t ret = mpu6050_calibrate(&offsets, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

void test_mpu6050_calibrate_single_sample(void) {
    mpu6050_calibration_t offsets;
    
    // Set mock data to known values
    mock_sensor_data.accelX = 100;
    mock_sensor_data.accelY = -200;
    mock_sensor_data.accelZ = 16384; // 1g on Z axis
    mock_sensor_data.gyroX = 50;
    mock_sensor_data.gyroY = -100;
    mock_sensor_data.gyroZ = 75;
    
    esp_err_t ret = mpu6050_calibrate(&offsets, 1);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(100, offsets.accel_offset[0]);
    TEST_ASSERT_EQUAL(-200, offsets.accel_offset[1]);
    TEST_ASSERT_EQUAL(16384, offsets.accel_offset[2]);
    TEST_ASSERT_EQUAL(50, offsets.gyro_offset[0]);
    TEST_ASSERT_EQUAL(-100, offsets.gyro_offset[1]);
    TEST_ASSERT_EQUAL(75, offsets.gyro_offset[2]);
}

void test_mpu6050_calibrate_multiple_samples(void) {
    mpu6050_calibration_t offsets;
    
    // Mock will return same values each time, so average should equal the value
    mock_sensor_data.accelX = 400;
    mock_sensor_data.accelY = -800;
    mock_sensor_data.accelZ = 16000;
    mock_sensor_data.gyroX = 200;
    mock_sensor_data.gyroY = -400;
    mock_sensor_data.gyroZ = 300;
    
    esp_err_t ret = mpu6050_calibrate(&offsets, 10);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(400, offsets.accel_offset[0]);
    TEST_ASSERT_EQUAL(-800, offsets.accel_offset[1]);
    TEST_ASSERT_EQUAL(16000, offsets.accel_offset[2]);
    TEST_ASSERT_EQUAL(200, offsets.gyro_offset[0]);
    TEST_ASSERT_EQUAL(-400, offsets.gyro_offset[1]);
    TEST_ASSERT_EQUAL(300, offsets.gyro_offset[2]);
}

void test_mpu6050_calibrate_read_failure(void) {
    mpu6050_calibration_t offsets;
    
    // Force read_data to fail
    mock_read_return_value = ESP_FAIL;
    
    esp_err_t ret = mpu6050_calibrate(&offsets, 10);
    
    TEST_ASSERT_EQUAL(ESP_FAIL, ret);
}

// ========== mpu6050_get_scale_factors Tests ==========

void test_mpu6050_get_scale_factors_null_data(void) {
    mpu6050_calibration_t offsets = {0};
    
    esp_err_t ret = mpu6050_get_scale_factors(NULL, &offsets);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

void test_mpu6050_get_scale_factors_null_offsets(void) {
    mpu6050_scaled_data_t data;
    
    esp_err_t ret = mpu6050_get_scale_factors(&data, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

void test_mpu6050_get_scale_factors_zero_offsets(void) {
    mpu6050_scaled_data_t scaled_data;
    mpu6050_calibration_t offsets = {0};
    
    // Set raw sensor data: 16384 = 1g, 131 = 1°/s
    mock_sensor_data.accelX = 16384;
    mock_sensor_data.accelY = -16384;
    mock_sensor_data.accelZ = 32768;
    mock_sensor_data.gyroX = 131;
    mock_sensor_data.gyroY = -131;
    mock_sensor_data.gyroZ = 262;
    
    esp_err_t ret = mpu6050_get_scale_factors(&scaled_data, &offsets);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, scaled_data.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -1.0, scaled_data.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 2.0, scaled_data.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, scaled_data.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -1.0, scaled_data.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 2.0, scaled_data.gyro_z);
}

void test_mpu6050_get_scale_factors_with_offsets(void) {
    mpu6050_scaled_data_t scaled_data;
    mpu6050_calibration_t offsets;
    
    // Set offsets
    offsets.accel_offset[0] = 100;
    offsets.accel_offset[1] = -50;
    offsets.accel_offset[2] = 200;
    offsets.gyro_offset[0] = 10;
    offsets.gyro_offset[1] = -20;
    offsets.gyro_offset[2] = 30;
    
    // Set raw sensor data
    mock_sensor_data.accelX = 16484;  // 16384 + 100 offset
    mock_sensor_data.accelY = -16434; // -16384 - 50 offset
    mock_sensor_data.accelZ = 32968;  // 32768 + 200 offset
    mock_sensor_data.gyroX = 141;     // 131 + 10 offset
    mock_sensor_data.gyroY = -151;    // -131 - 20 offset
    mock_sensor_data.gyroZ = 292;     // 262 + 30 offset
    
    esp_err_t ret = mpu6050_get_scale_factors(&scaled_data, &offsets);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    // After offset correction: raw - offset should give us back the base values
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, scaled_data.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -1.0, scaled_data.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 2.0, scaled_data.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, scaled_data.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -1.0, scaled_data.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 2.0, scaled_data.gyro_z);
}

void test_mpu6050_get_scale_factors_accel_accuracy(void) {
    mpu6050_scaled_data_t scaled_data;
    mpu6050_calibration_t offsets = {0};
    
    // Test accelerometer scale: 16384 LSB/g for ±2g range
    mock_sensor_data.accelX = 8192;   // 0.5g
    mock_sensor_data.accelY = 4096;   // 0.25g
    mock_sensor_data.accelZ = 0;      // 0g
    mock_sensor_data.gyroX = 0;
    mock_sensor_data.gyroY = 0;
    mock_sensor_data.gyroZ = 0;
    
    esp_err_t ret = mpu6050_get_scale_factors(&scaled_data, &offsets);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.5, scaled_data.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.25, scaled_data.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, scaled_data.accel_z);
}

void test_mpu6050_get_scale_factors_gyro_accuracy(void) {
    mpu6050_scaled_data_t scaled_data;
    mpu6050_calibration_t offsets = {0};
    
    // Test gyroscope scale: 131 LSB/(°/s) for ±250°/s range
    mock_sensor_data.accelX = 0;
    mock_sensor_data.accelY = 0;
    mock_sensor_data.accelZ = 0;
    mock_sensor_data.gyroX = 655;     // 5°/s
    mock_sensor_data.gyroY = 1310;    // 10°/s
    mock_sensor_data.gyroZ = -655;    // -5°/s
    
    esp_err_t ret = mpu6050_get_scale_factors(&scaled_data, &offsets);
    
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 5.0, scaled_data.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 10.0, scaled_data.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(0.1, -5.0, scaled_data.gyro_z);
}

void test_mpu6050_get_scale_factors_read_failure(void) {
    mpu6050_scaled_data_t scaled_data;
    mpu6050_calibration_t offsets = {0};
    
    // Force read_data to fail
    mock_read_return_value = ESP_ERR_TIMEOUT;
    
    esp_err_t ret = mpu6050_get_scale_factors(&scaled_data, &offsets);
    
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, ret);
}

void test_mpu6050_get_scale_factors_negative_values(void) {
    mpu6050_scaled_data_t scaled_data;
    mpu6050_calibration_t offsets = {0};

    // Test with all negative values
    mock_sensor_data.accelX = -16384;
    mock_sensor_data.accelY = -8192;
    mock_sensor_data.accelZ = -4096;
    mock_sensor_data.gyroX = -131;
    mock_sensor_data.gyroY = -262;
    mock_sensor_data.gyroZ = -655;

    esp_err_t ret = mpu6050_get_scale_factors(&scaled_data, &offsets);

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -1.0, scaled_data.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -0.5, scaled_data.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -0.25, scaled_data.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -1.0, scaled_data.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01, -2.0, scaled_data.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(0.1, -5.0, scaled_data.gyro_z);
}

// Main test runner
int main(void) {
    UNITY_BEGIN();

    // mpu6050_calibrate tests
    RUN_TEST(test_mpu6050_calibrate_null_offsets);
    RUN_TEST(test_mpu6050_calibrate_zero_samples);
    RUN_TEST(test_mpu6050_calibrate_single_sample);
    RUN_TEST(test_mpu6050_calibrate_multiple_samples);
    RUN_TEST(test_mpu6050_calibrate_read_failure);

    // mpu6050_get_scale_factors tests
    RUN_TEST(test_mpu6050_get_scale_factors_null_data);
    RUN_TEST(test_mpu6050_get_scale_factors_null_offsets);
    RUN_TEST(test_mpu6050_get_scale_factors_zero_offsets);
    RUN_TEST(test_mpu6050_get_scale_factors_with_offsets);
    RUN_TEST(test_mpu6050_get_scale_factors_accel_accuracy);
    RUN_TEST(test_mpu6050_get_scale_factors_gyro_accuracy);
    RUN_TEST(test_mpu6050_get_scale_factors_read_failure);
    RUN_TEST(test_mpu6050_get_scale_factors_negative_values);

    return UNITY_END();
}