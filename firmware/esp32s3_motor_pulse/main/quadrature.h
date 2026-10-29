#pragma once

#include <stdint.h>

// Returns one signed count per valid A/B edge; skipped or unchanged states count zero.
static inline int8_t quadrature_step(uint8_t previous, uint8_t current)
{
    static const int8_t transitions[16] = {
         0, +1, -1,  0,
        -1,  0,  0, +1,
        +1,  0,  0, -1,
         0, -1, +1,  0,
    };
    return transitions[((previous & 3U) << 2) | (current & 3U)];
}
