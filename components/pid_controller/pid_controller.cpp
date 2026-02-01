/*
 * PID controller component for ESP32 Drone
 */

#include "pid_controller.h"

void pid_controller_init(flight_pid_controllers_t *pids) {
     // Initialize roll PID
    pids->roll.kp = 1.0f;
    pids->roll.ki = 0.1f;
    pids->roll.kd = 0.05f;
    pids->roll.integral = 0.0f;
    pids->roll.prev_error = 0.0f;
    pids->roll.output_min = -100.0f;
    pids->roll.output_max = 100.0f;

    // Initialize pitch PID
    pids->pitch.kp = 1.0f;
    pids->pitch.ki = 0.1f;
    pids->pitch.kd = 0.05f;
    pids->pitch.integral = 0.0f;
    pids->pitch.prev_error = 0.0f;
    pids->pitch.output_min = -100.0f;
    pids->pitch.output_max = 100.0f;

    // Initialize yaw PID
    pids->yaw.kp = 1.0f;
    pids->yaw.ki = 0.1f;
    pids->yaw.kd = 0.05f;
    pids->yaw.integral = 0.0f;
    pids->yaw.prev_error = 0.0f;
    pids->yaw.output_min = -100.0f;
    pids->yaw.output_max = 100.0f;
}

float pid_controller_compute(pid_controller_t *pid, float setpoint, float measured, float dt, uint32_t current_time) {
    float error = setpoint - measured;

    // Proportional term
    float Pout = pid->kp * error;

    // Integral term
    pid->integral += error * dt;
    float Iout = pid->ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float Dout = pid->kd * derivative;

    // Compute total output
    float output = Pout + Iout + Dout;

    // Output limiting
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    // Save error and timestamp for next iteration
    pid->prev_error = error;
    pid->last_update_time = current_time;

    return output;
}

void pid_controller_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_update_time = 0;
}
