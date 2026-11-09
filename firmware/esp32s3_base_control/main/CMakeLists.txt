idf_component_register(SRCS "main.c" "control.c" "protocol.c"
    INCLUDE_DIRS "." "../../esp32s3_motor_pulse/main"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_timer)

# 硬件配置切换（见 board_config.h）：把 CMake 变量转成 C 编译宏
#   idf.py -DDRIVER_WSDC2412D=0 build            → 旧 AT8236 双 PWM
#   idf.py -DDRIVER_WSDC2412D=1 build            → 新 WSDC2412D（默认）
foreach(_opt DRIVER_WSDC2412D WHEEL_CIRC_MM COUNTS_PER_REV)
    if(DEFINED ${_opt})
        target_compile_definitions(${COMPONENT_LIB} PRIVATE ${_opt}=${${_opt}})
    endif()
endforeach()
