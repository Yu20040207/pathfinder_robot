#include <assert.h>

#include "../main/motor_ramp.h"

int main(void)
{
    assert(motor_ramp_duty(0, 40) == 0);
    assert(motor_ramp_duty(10, 40) == 256);
    assert(motor_ramp_duty(20, 40) == 512);
    assert(motor_ramp_duty(30, 40) == 256);
    assert(motor_ramp_duty(40, 40) == 0);
    assert(motor_ramp_duty(41, 40) == 0);
    assert(motor_ramp_duty(0, 150) == 0);
    assert(motor_ramp_duty(75, 150) == 512);
    assert(motor_ramp_duty(150, 150) == 0);
    assert(motor_start_ramp_duty(0) == 512);
    assert(motor_start_ramp_duty(10) == 512);
    assert(motor_start_ramp_duty(20) == 410);
    assert(motor_start_ramp_duty(80) == 512);
    assert(motor_start_ramp_duty(140) == 410);
    assert(motor_start_ramp_duty(150) == 0);
    return 0;
}
