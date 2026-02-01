idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050 pid_controller attitude_estimation esp_timer)