#include <assert.h>
#include "protocol.h"

int main(void) {
    float left=0, right=0;
    assert(parse_velocity("v -300 300", &left, &right));
    assert(left==-300 && right==300);
    assert(!parse_velocity("v 300.01 0", &left, &right));
    assert(!parse_velocity("v nan 0", &left, &right));
    assert(!parse_velocity("v 10", &left, &right));
    assert(!parse_velocity("v 10 20 extra", &left, &right));
    assert(protocol_is_v2_handshake("BASE_CONTROL_V2"));
    assert(!protocol_is_v2_handshake("BASE_CONTROL_V1"));
    assert(!protocol_is_v2_handshake("BASE_CONTROL_V2 extra"));
    return 0;
}
