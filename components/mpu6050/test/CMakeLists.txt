idf_component_register(
    SRCS "mpu6050_test.cpp"
    INCLUDE_DIRS "."
    REQUIRES unity mpu6050
)

# Enable function wrapping for mocking mpu6050_read_data
target_link_options(${COMPONENT_LIB} PRIVATE
    "-Wl,--wrap=mpu6050_read_data"
)