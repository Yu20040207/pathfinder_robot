#pragma once

// Forward directions after the wheels were reassembled on 2026-10-16.
typedef enum { MOTOR_LEFT, MOTOR_RIGHT } motor_side_t;
typedef enum { MOTOR_FORWARD, MOTOR_REVERSE } motor_direction_t;

// LEDC channels follow GPIO4=AIN1, GPIO5=AIN2, GPIO6=BIN1, GPIO7=BIN2.
static inline int motor_channel(motor_side_t side, motor_direction_t direction)
{
    if (side == MOTOR_LEFT) {
        return direction == MOTOR_FORWARD ? 0 : 1;
    }
    return direction == MOTOR_FORWARD ? 3 : 2;
}
