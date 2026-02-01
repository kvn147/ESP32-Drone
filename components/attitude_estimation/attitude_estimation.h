/*
 * Header file for attitude estimation component
 */
#ifndef ATTITUDE_ESTIMATION_H
#define ATTITUDE_ESTIMATION_H

#include <cstdint>
#include <cmath>

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct {
    float alpha; // Complementary filter coefficient
    attitude_t attitude; // Current estimated attitude
    uint32_t last_update_time; // Timestamp of the last update  
} attitude_estimator_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the attitude estimation component
 */
void attitude_estimation_init(attitude_estimator_t *estimator, float alpha);

/**
 * @brief Update the attitude estimate with new IMU data
 * @param estimator Pointer to the attitude estimator structure
 * @param accel Array of accelerometer values [x, y, z] in g
 * @param gyro Array of gyroscope values [x, y, z] in degrees/sec
 * @param current_time Current timestamp in milliseconds
 */
void attitude_estimation_update(attitude_estimator_t *estimator, const float accel[3], const float gyro[3], uint32_t current_time);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_ESTIMATION_H
