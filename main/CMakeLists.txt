idf_component_register(
    SRCS
        "main.c"
        "motors.c"
        "encoders.c"
        "pid.c"
        "odometry.c"
        "imu.c"
        "esp32_serial_transport.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver micro_ros_espidf_component
)
