#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { BASE_OK=0, BASE_STALL=1, BASE_DIRECTION=2, BASE_OVERSPEED=3,
       BASE_ENCODER=4, BASE_TIMING=5, BASE_TRACKING=6 };
typedef struct {
    float target[2], setpoint[2], measured[2], integral[2], pwm[2];
    int64_t last_us, deadline_us, turn_started_us;
    float stall_s[2], wrong_s[2], fast_s[2], tracking_s[2], still_s[2];
    float startup_s[2];
    float sync_distance[2];
    float turn_history_mm[2][5], turn_recent_mm[2];
    unsigned turn_ring_pos;
    bool startup_done[2];
    int32_t counts[2];
    uint32_t skipped[2];
    bool active;
    bool duty_mode;
    bool loaded_mode;
    bool arc_mode;
    bool sync_mode;
    bool turn_mode;
    int fault;
} base_control_t;
void base_init(base_control_t *c, int64_t now, int32_t left, int32_t right);
bool base_command(base_control_t *c, float left_mm_s, float right_mm_s, int64_t now);
bool base_duty_command(base_control_t *c, float left_duty, float right_duty, int64_t now);
bool base_loaded_command(base_control_t *c, float left_mm_s, float right_mm_s, int64_t now);
bool base_arc_command(base_control_t *c, float right_minus_left_mm_s, int64_t now);
bool base_sync_command(base_control_t *c, int direction, int64_t now);
bool base_turn_command(base_control_t *c, int direction, int64_t now);
void base_stop(base_control_t *c);
void base_tick(base_control_t *c, int64_t now, int32_t left, int32_t right,
               uint32_t left_skipped, uint32_t right_skipped);
