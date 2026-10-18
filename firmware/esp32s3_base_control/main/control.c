#include "control.h"
#include <math.h>
#include <string.h>

static float clamp(float x, float low, float high) {
    return fminf(high, fmaxf(low, x));
}
static float approach(float x, float goal, float step) {
    return x + clamp(goal-x, -step, step);
}
void base_stop(base_control_t *c) {
    c->active = false;
    c->duty_mode = false;
    c->loaded_mode = false;
    c->arc_mode = false;
    c->sync_mode = false;
    c->turn_mode = false;
    c->deadline_us = 0;
    c->turn_started_us = 0;
    c->turn_ring_pos = 0;
    for (int i=0; i<2; ++i) {
        c->target[i] = c->setpoint[i] = c->integral[i] = c->pwm[i] = 0;
        c->stall_s[i] = c->wrong_s[i] = c->fast_s[i] = 0;
        c->tracking_s[i] = 0;
        c->startup_s[i] = 0;
        c->sync_distance[i] = 0;
        c->turn_recent_mm[i] = 0;
        for (int j=0; j<5; ++j) c->turn_history_mm[i][j] = 0;
        c->startup_done[i] = false;
    }
}
void base_init(base_control_t *c, int64_t now, int32_t left, int32_t right) {
    memset(c, 0, sizeof(*c));
    c->last_us = now;
    c->counts[0] = left;
    c->counts[1] = right;
}
bool base_command(base_control_t *c, float left, float right, int64_t now) {
    if (!isfinite(left) || !isfinite(right) || fabsf(left)>100 || fabsf(right)>100 || c->fault) {
        base_stop(c);
        return false;
    }
    if (left==0 && right==0) { base_stop(c); return true; }
    if (c->duty_mode || c->loaded_mode || c->arc_mode || c->sync_mode) base_stop(c);
    if (!c->active) {
        for (int i=0; i<2; ++i) {
            c->startup_s[i] = 0;
            c->startup_done[i] = c->measured[i] * (i==0 ? left : right) < 0 &&
                                 fabsf(c->measured[i]) > 15.0f;
        }
    }
    c->target[0] = left;
    c->target[1] = right;
    c->deadline_us = now + 300000;
    c->active = true;
    return true;
}
bool base_duty_command(base_control_t *c, float left, float right, int64_t now) {
    if (!isfinite(left) || !isfinite(right) || left<0 || right<0 ||
        left>.65f || right>.65f || c->fault) {
        base_stop(c);
        return false;
    }
    if (left==0 && right==0) { base_stop(c); return true; }
    if (c->active && !c->duty_mode) base_stop(c);
    c->duty_mode = true;
    c->active = true;
    c->target[0] = left;
    c->target[1] = right;
    c->deadline_us = now + 300000;
    return true;
}
bool base_loaded_command(base_control_t *c, float left, float right, int64_t now) {
    if (!isfinite(left) || !isfinite(right) || left<150 || right<150 ||
        left>200 || right>200 || c->fault) {
        base_stop(c);
        return false;
    }
    if (c->active && !c->loaded_mode) base_stop(c);
    c->loaded_mode = true;
    c->active = true;
    c->target[0] = left;
    c->target[1] = right;
    c->deadline_us = now + 300000;
    return true;
}
bool base_arc_command(base_control_t *c, float delta, int64_t now) {
    if (!isfinite(delta) || fabsf(delta)>50 || c->fault) {
        base_stop(c);
        return false;
    }
    if (c->active && !c->arc_mode) base_stop(c);
    c->arc_mode = true;
    c->active = true;
    c->target[0] = 0;
    c->target[1] = delta;
    c->deadline_us = now + 300000;
    return true;
}
static bool sync_wheels(base_control_t *c, int left, int right, bool turn, int64_t now) {
    if ((left!=1 && left!=-1) || (right!=1 && right!=-1) || c->fault) {
        base_stop(c);
        return false;
    }
    if (c->active && (!c->sync_mode || c->turn_mode!=turn ||
                      c->target[0]!=left || c->target[1]!=right)) base_stop(c);
    if (!c->active && turn) c->turn_started_us = now;
    c->sync_mode = true;
    c->turn_mode = turn;
    c->active = true;
    c->target[0] = left;
    c->target[1] = right;
    c->deadline_us = now + 300000;
    return true;
}
bool base_sync_command(base_control_t *c, int direction, int64_t now) {
    return sync_wheels(c, direction, direction, false, now);
}
bool base_turn_command(base_control_t *c, int direction, int64_t now) {
    if (direction!=1 && direction!=-1) {
        base_stop(c);
        return false;
    }
    return sync_wheels(c, -direction, direction, true, now);
}
static void fail(base_control_t *c, int fault) {
    c->fault = fault;
    base_stop(c);
}
void base_tick(base_control_t *c, int64_t now, int32_t left, int32_t right,
               uint32_t left_skipped, uint32_t right_skipped) {
    const int32_t current[2] = {left, right};
    const uint32_t skips[2] = {left_skipped, right_skipped};
    const float dt = (now-c->last_us)/1000000.0f;
    if (dt<=0) { fail(c, BASE_TIMING); return; }
    if (c->active && now>=c->deadline_us) base_stop(c);
    if (c->active && c->turn_mode && now-c->turn_started_us>=500000)
        fail(c, BASE_TIMING);
    if (c->active && dt>.1f) fail(c, BASE_TIMING);
    c->last_us = now;
    int32_t deltas[2];
    for (int i=0; i<2; ++i) {
        // Subtract via unsigned arithmetic to handle int32 encoder wrap.
        int32_t delta = (int32_t)((uint32_t)current[i]-(uint32_t)c->counts[i]);
        deltas[i] = delta;
        float speed = (i==0 ? -1.0f : 1.0f)*delta*(210.0f/1468.0f)/dt;
        c->measured[i] += (1.0f-expf(-dt/.08f))*(speed-c->measured[i]);
        if (c->active && (uint32_t)(skips[i]-c->skipped[i])>8) fail(c, BASE_ENCODER);
        if (c->active && c->sync_mode)
            c->sync_distance[i] += c->target[i]*(i==0 ? -1.0f : 1.0f)*
                                   delta*(210.0f/1468.0f);
        if (c->active && c->turn_mode) {
            float recent_step = c->target[i]*(i==0 ? -1.0f : 1.0f)*
                                delta*(210.0f/1468.0f);
            c->turn_recent_mm[i] += recent_step-
                                    c->turn_history_mm[i][c->turn_ring_pos];
            c->turn_history_mm[i][c->turn_ring_pos] = recent_step;
        }
        c->counts[i] = current[i];
        c->skipped[i] = skips[i];
    }
    if (c->active && c->turn_mode) c->turn_ring_pos = (c->turn_ring_pos+1)%5;
    for (int i=0; i<2; ++i) {
        int32_t delta = deltas[i];
        if (!c->active || c->fault) { c->pwm[i] = 0; continue; }
        if (c->sync_mode) {
            float direction = c->target[i];
            float own = direction*c->measured[i];
            float other = c->target[1-i]*c->measured[1-i];
            float lag = other-own;
            float distance_lag = c->sync_distance[1-i]-c->sync_distance[i];
            c->startup_s[i] += dt;
            c->setpoint[i] = direction*180.0f;
            float desired = clamp(.6f+.01f*distance_lag, .6f, .75f);
            bool breakaway = c->turn_mode && c->startup_s[i]>=.15f &&
                             c->turn_recent_mm[i]<.5f;
            float duty = breakaway ? .75f :
                         approach(fmaxf(.6f, fabsf(c->pwm[i])), desired, .5f*dt);
            c->pwm[i] = direction*duty;
            c->stall_s[i] = c->startup_s[i]>.2f && delta==0 ? c->stall_s[i]+dt : 0;
            c->wrong_s[i] = own<-20 ? c->wrong_s[i]+dt : 0;
            c->fast_s[i] = fabsf(c->measured[i])>300 ? c->fast_s[i]+dt : 0;
            c->tracking_s[i] = c->turn_mode ?
                (c->startup_s[i]>.2f && fabsf(distance_lag)>8 ? c->tracking_s[i]+dt : 0) :
                (c->startup_s[i]>.3f && fabsf(lag)>40 ? c->tracking_s[i]+dt : 0);
            if (c->turn_mode && c->startup_s[i]>=.35f &&
                c->turn_recent_mm[i]<.5f) fail(c, BASE_STALL);
            else if (c->stall_s[i]>.35f) fail(c, BASE_STALL);
            else if (c->wrong_s[i]>.2f) fail(c, BASE_DIRECTION);
            else if (c->fast_s[i]>.1f) fail(c, BASE_OVERSPEED);
            else if (c->tracking_s[i]>(c->turn_mode ? .1f : .4f)) fail(c, BASE_TRACKING);
            continue;
        }
        if (c->duty_mode) {
            // Forward-only diagnostic. Existing encoder fault, stall and
            // overspeed latches remain in force during fixed-duty tests.
            c->setpoint[i] = 0;
            c->pwm[i] = c->target[i];
            c->stall_s[i] = c->pwm[i]>=.3f && delta==0 ? c->stall_s[i]+dt : 0;
            c->wrong_s[i] = c->measured[i]<-20 ? c->wrong_s[i]+dt : 0;
            c->fast_s[i] = fabsf(c->measured[i])>300 ? c->fast_s[i]+dt : 0;
            if (c->stall_s[i]>.75f) fail(c, BASE_STALL);
            else if (c->wrong_s[i]>.2f) fail(c, BASE_DIRECTION);
            else if (c->fast_s[i]>.1f) fail(c, BASE_OVERSPEED);
            continue;
        }
        if (c->loaded_mode) {
            // Temporary 2.5 kg loaded-floor experiment. Both wheels must
            // stay in the empirically moving duty interval; this mode is
            // deliberately separate from the navigation velocity command.
            c->startup_s[i] += dt;
            c->setpoint[i] = c->target[i];
            float error = c->target[i]-c->measured[i];
            c->integral[i] = clamp(c->integral[i]+.001f*error*dt, 0, .05f);
            c->pwm[i] = clamp(.6f + .0008f*error + c->integral[i], .6f, .65f);
            c->stall_s[i] = c->startup_s[i]>.2f && c->measured[i]<20 ? c->stall_s[i]+dt : 0;
            c->wrong_s[i] = c->measured[i]<-20 ? c->wrong_s[i]+dt : 0;
            c->fast_s[i] = fabsf(c->measured[i])>300 ? c->fast_s[i]+dt : 0;
            c->tracking_s[i] = c->startup_s[i]>.3f && fabsf(error)>40 ?
                               c->tracking_s[i]+dt : 0;
            if (c->stall_s[i]>.35f) fail(c, BASE_STALL);
            else if (c->wrong_s[i]>.2f) fail(c, BASE_DIRECTION);
            else if (c->fast_s[i]>.1f) fail(c, BASE_OVERSPEED);
            else if (c->tracking_s[i]>.4f) fail(c, BASE_TRACKING);
            continue;
        }
        if (c->arc_mode) {
            // Arc diagnostic regulates right-minus-left wheel speed. The
            // slower wheel remains at the empirically moving 60% floor.
            if (i==0) c->startup_s[0] += dt;
            float delta_goal = c->target[1];
            bool fast_wheel = (delta_goal>0 && i==1) || (delta_goal<0 && i==0);
            float actual_delta = c->measured[1]-c->measured[0];
            float error = delta_goal>0 ? delta_goal-actual_delta : actual_delta-delta_goal;
            if (fast_wheel) {
                c->integral[0] = clamp(c->integral[0]+.003f*error*dt, 0, .05f);
                c->pwm[i] = clamp(.6f+.0015f*error+c->integral[0], .6f, .65f);
            } else c->pwm[i] = .6f;
            c->setpoint[i] = i==0 ? 0 : delta_goal;
            c->stall_s[i] = c->startup_s[0]>.2f && c->measured[i]<20 ? c->stall_s[i]+dt : 0;
            c->wrong_s[i] = c->measured[i]<-20 ? c->wrong_s[i]+dt : 0;
            c->fast_s[i] = fabsf(c->measured[i])>300 ? c->fast_s[i]+dt : 0;
            if (c->stall_s[i]>.35f) fail(c, BASE_STALL);
            else if (c->wrong_s[i]>.2f) fail(c, BASE_DIRECTION);
            else if (c->fast_s[i]>.1f) fail(c, BASE_OVERSPEED);
            if (i==1 && c->active) {
                c->tracking_s[0] = c->startup_s[0]>.3f &&
                    fabsf(delta_goal-(c->measured[1]-c->measured[0]))>25 ?
                    c->tracking_s[0]+dt : 0;
                if (c->tracking_s[0]>.4f) fail(c, BASE_TRACKING);
            }
            continue;
        }
        float goal = c->target[i];
        // Coast through zero before reversing the electrical direction.
        if (goal*c->setpoint[i]<0 || (fabsf(c->setpoint[i])<.01f && goal*c->measured[i]<0 && fabsf(c->measured[i])>15))
            goal = 0;
        c->setpoint[i] = approach(c->setpoint[i], goal, 150.0f*dt);
        if (fabsf(c->setpoint[i])<.01f) {
            c->pwm[i]=c->integral[i]=c->stall_s[i]=c->wrong_s[i]=c->fast_s[i]=0;
            continue;
        }
        float direction = c->setpoint[i]>0 ? 1.0f : -1.0f;
        if (!c->startup_done[i]) {
            if (goal*direction <= 0 || direction*c->measured[i] >= 15.0f ||
                c->startup_s[i] >= .20f) {
                c->startup_done[i] = true;
            } else {
                c->startup_s[i] += dt;
                // Start at the proven suspended-wheel duty. Only a wheel
                // still below 15 mm/s receives the higher loaded-start duty.
                c->pwm[i] = direction*clamp(.5f + 1.25f*c->startup_s[i], .5f, .75f);
                c->stall_s[i] = delta==0 ? c->stall_s[i]+dt : 0;
                c->wrong_s[i] = direction*c->measured[i]<-20 ? c->wrong_s[i]+dt : 0;
                c->fast_s[i] = fabsf(c->measured[i])>300 ? c->fast_s[i]+dt : 0;
                if (c->stall_s[i]>.75f) fail(c, BASE_STALL);
                else if (c->wrong_s[i]>.2f) fail(c, BASE_DIRECTION);
                else if (c->fast_s[i]>.1f) fail(c, BASE_OVERSPEED);
                continue;
            }
        }
        float desired = fabsf(c->setpoint[i]);
        float error = desired-direction*c->measured[i];
        float trial_i = clamp(c->integral[i]+.0015f*error*dt, 0, .75f);
        float feedforward = .22f + .0017f*desired;
        float effort = feedforward + .0012f*error + trial_i;
        if (effort<=.75f || error<0) c->integral[i] = trial_i;
        effort = clamp(feedforward + .0012f*error + c->integral[i], 0, .75f);
        // Preserve wheel momentum after the startup pulse: an abrupt duty
        // reduction from the startup duty can stop loaded wheels.
        // Stop/fault paths still bypass this ramp and set PWM to zero.
        float previous = fabsf(c->pwm[i]);
        if (direction*c->measured[i] <= desired && effort < previous)
            effort = previous;
        float rise_rate = error > 20.0f ? 3.0f : 1.0f;
        c->pwm[i] = direction*approach(previous, effort,
                (effort < previous ? .2f : rise_rate)*dt);
        c->stall_s[i] = fabsf(c->pwm[i])>=.3f && delta==0 ? c->stall_s[i]+dt : 0;
        c->wrong_s[i] = direction*c->measured[i]<-20 ? c->wrong_s[i]+dt : 0;
        c->fast_s[i] = fabsf(c->measured[i])>300 ? c->fast_s[i]+dt : 0;
        if (c->stall_s[i]>.75f) fail(c, BASE_STALL);
        else if (c->wrong_s[i]>.2f) fail(c, BASE_DIRECTION);
        else if (c->fast_s[i]>.1f) fail(c, BASE_OVERSPEED);
    }
}
