#include <assert.h>

#include "../main/quadrature.h"

int main(void)
{
    const uint8_t forward[] = {0, 1, 3, 2, 0};
    const uint8_t reverse[] = {0, 2, 3, 1, 0};
    int forward_count = 0;
    int reverse_count = 0;
    for (int i = 1; i < 5; ++i) {
        forward_count += quadrature_step(forward[i - 1], forward[i]);
        reverse_count += quadrature_step(reverse[i - 1], reverse[i]);
    }
    assert(forward_count == 4);
    assert(reverse_count == -4);
    assert(quadrature_step(0, 0) == 0);
    assert(quadrature_step(0, 3) == 0);
    assert(quadrature_step(3, 0) == 0);
    return 0;
}
