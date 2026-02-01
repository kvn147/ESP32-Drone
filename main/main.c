#include <stdio.h>
#include "mpu6050.h"
#include "attitude_estimation.h"
#include "pid_controller.h"

void app_main(void) {
  mpu6050_init();
  mpu6050_configure(0x00, 0x00); // Example configuration

  mpu6050_calibration_t calibration;

  attitude_estimator_t estimator;
  attitude_estimation_init(&estimator, 0.98f);

  flight_pid_controllers_t pids;
  pid_controller_init(&pids);

  // FreeRTOS Queues
  QueueHandle_t imu_queue = xQueueCreate(10, sizeof(mpu6050_scaled_data_t));

  // Tasks
  xTaskCreate(imu_task, "IMU", 4096, &estimator, 5, NULL);
  xTaskCreate(attitude_task, "Attitude", 4096, &estimator, 5, NULL);
  xTaskCreate(control_task, "Control", 4096, &pids, 5, NULL);
}