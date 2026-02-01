/*
 * Header file for PID controller component
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cstdint>

typedef struct {
    float kp, ki, kd; // PID coefficients
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    uint32_t last_update_time;
} pid_controller_t;

typedef struct {
    pid_controller_t roll;
    pid_controller_t pitch;
    pid_controller_t yaw;
} flight_pid_controllers_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the PID controller component
 */
void pid_controller_init(flight_pid_controllers_t *pids);

float pid_controller_compute(pid_controller_t *pid, float setpoint, float measured, float dt, uint32_t current_time);

void pid_controller_reset(pid_controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
