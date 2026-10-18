idf_component_register(SRCS "main.c" "control.c" "protocol.c"
    INCLUDE_DIRS "." "../../esp32s3_motor_pulse/main"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_timer)
