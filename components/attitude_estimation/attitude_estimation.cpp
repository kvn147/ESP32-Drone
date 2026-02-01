/*
 * Attitude estimation component for ESP32 Drone
 */

#include "attitude_estimation.h"

void attitude_estimation_init(attitude_estimator_t *estimator, float alpha) {
    estimator->alpha = alpha;
    estimator->attitude.roll = 0.0f;
    estimator->attitude.pitch = 0.0f;
    estimator->attitude.yaw = 0.0f;
    estimator->last_update_time = 0; // Initialize timestamp
}

void update_attitude_estimation(attitude_estimator_t *estimator, float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, uint32_t current_time) {
    // Implementation of attitude estimation algorithm using complementary filter

    float accel_roll = atan2f(accelY, accelZ) * 180.0f / M_PI;
    float accel_pitch = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * 180.0f / M_PI;
    float dt = (current_time - estimator->last_update_time) / 1000.0f; // Convert ms to seconds
    
    estimator->attitude.roll = estimator->alpha * (estimator->attitude.roll + gyroX * dt) + (1.0f - estimator->alpha) * accel_roll;
    estimator->attitude.pitch = estimator->alpha * (estimator->attitude.pitch + gyroY * dt) + (1.0f - estimator->alpha) * accel_pitch; 
    estimator->attitude.yaw += gyroZ * dt;

    estimator->last_update_time = current_time;
}

void attitude_estimation_update(attitude_estimator_t *estimator, const float accel[3], const float gyro[3], uint32_t current_time) {
    update_attitude_estimation(estimator, gyro[0], gyro[1], gyro[2],
                               accel[0], accel[1], accel[2], current_time);
}
