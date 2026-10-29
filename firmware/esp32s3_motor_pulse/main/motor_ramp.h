#pragma once

#include <stdint.h>

// Even number of 100 ms intervals: 0 -> 50% -> 0 (10-bit PWM).
static inline uint32_t motor_ramp_duty(uint32_t step, uint32_t total_steps)
{
    if (total_steps < 2 || (total_steps & 1U) != 0 || step >= total_steps) {
        return 0;
    }
    const uint32_t half = total_steps / 2U;
    return step <= half ? 512U * step / half : 512U * (total_steps - step) / half;
}

// Fifteen-second diagnostic profile: 1 s at 50% to overcome static friction,
// then a 40% <-> 50% sweep before stopping. One step is 100 ms.
static inline uint32_t motor_start_ramp_duty(uint32_t step)
{
    if (step <= 10) {
        return 512;
    }
    if (step <= 20) {
        return 512U - 102U * (step - 10U) / 10U;
    }
    if (step <= 80) {
        return 410U + 102U * (step - 20U) / 60U;
    }
    if (step <= 140) {
        return 512U - 102U * (step - 80U) / 60U;
    }
    if (step <= 150) {
        return 410U * (150U - step) / 10U;
    }
    return 0;
}
