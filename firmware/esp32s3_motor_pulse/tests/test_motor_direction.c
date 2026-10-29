#include <assert.h>

#include "../main/motor_direction.h"

int main(void)
{
    assert(motor_channel(MOTOR_LEFT, MOTOR_FORWARD) == 0);  // AIN1
    assert(motor_channel(MOTOR_LEFT, MOTOR_REVERSE) == 1);  // AIN2
    assert(motor_channel(MOTOR_RIGHT, MOTOR_FORWARD) == 3); // BIN2
    assert(motor_channel(MOTOR_RIGHT, MOTOR_REVERSE) == 2); // BIN1
    return 0;
}
