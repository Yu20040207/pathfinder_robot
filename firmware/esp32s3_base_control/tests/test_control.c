#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "control.h"
#include "protocol.h"

static void tick(base_control_t *c, int64_t now, int left, int right) {
    base_tick(c, now, left, right, 0, 0);
}
static void loaded_plant_run(float left_goal, float right_goal) {
    base_control_t c;
    base_init(&c, 0, 0, 0);
    float physical[2]={0, 0}, count_f[2]={0, 0};
    for (int i=1; i<=100; ++i) {
        int64_t now=i*20000LL;
        if (i%5==1) assert(base_command(&c, left_goal, right_goal, now));
        for (int wheel=0; wheel<2; ++wheel) {
            float duty=c.pwm[wheel];
            float threshold=wheel==0 ? .58f : .55f;
            float target=copysignf(fmaxf(0, fabsf(duty)-threshold)*700, duty);
            physical[wheel] += .25f*(target-physical[wheel]);
            count_f[wheel] += physical[wheel]*.02f/(210.0f/1468.0f) * (wheel==0 ? -1 : 1);
        }
        tick(&c, now, (int)lroundf(count_f[0]), (int)lroundf(count_f[1]));
        assert(c.fault==BASE_OK && c.active);
        assert(fabsf(c.pwm[0])<=.75f && fabsf(c.pwm[1])<=.75f);
    }
    assert(left_goal*c.measured[0]>40 && right_goal*c.measured[1]>40);
    base_stop(&c);
    assert(c.pwm[0]==0 && c.pwm[1]==0);
}
int main(void) {
    base_control_t c;
    // Loaded diagnostic is forward-only and cannot silently accept a low
    // target that the measured 60% duty floor cannot produce.
    base_init(&c, 0, 0, 0);
    assert(!base_loaded_command(&c, 100, 160, 0));
    assert(!c.active);
    assert(base_loaded_command(&c, 160, 160, 0));
    tick(&c, 20000, -8, 8);
    assert(c.loaded_mode && c.pwm[0]>=.6f && c.pwm[0]<=.65f);
    assert(c.pwm[1]>=.6f && c.pwm[1]<=.65f);
    for (int i=2; i<30; ++i) {
        base_loaded_command(&c, 160, 160, i*20000LL);
        tick(&c, i*20000LL, -8, 8);
    }
    assert(c.fault==BASE_STALL && !c.active && c.pwm[0]==0 && c.pwm[1]==0);
    base_init(&c, 0, 0, 0);
    assert(base_loaded_command(&c, 160, 180, 0));
    tick(&c, 20000, -8, 8);
    tick(&c, 320000, -8, 8);
    assert(!c.active && !c.loaded_mode && c.pwm[0]==0 && c.pwm[1]==0);
    // Two unequal, loaded wheel plants should remain moving and create the
    // requested speed difference without leaving the measured duty band.
    base_init(&c, 0, 0, 0);
    float bl=0, br=0, bpl=0, bpr=0;
    for (int i=1; i<=70; ++i) {
        if (i%5==1) assert(base_loaded_command(&c, 160, 180, i*20000LL));
        bl += .15f*(270*c.pwm[0]-bl);
        br += .15f*(290*c.pwm[1]-br);
        bpl += bl*.02f/(210.0f/1468.0f);
        bpr += br*.02f/(210.0f/1468.0f);
        tick(&c, i*20000LL, -(int)lroundf(bpl), (int)lroundf(bpr));
        assert(c.fault==BASE_OK && c.pwm[0]>=.6f && c.pwm[0]<=.65f);
        assert(c.pwm[1]>=.6f && c.pwm[1]<=.65f);
    }
    assert(c.measured[1]>c.measured[0]+10);
    // A loaded command that keeps both wheels moving at nearly the same
    // speed despite different targets must fail instead of reporting OK.
    base_init(&c, 0, 0, 0);
    int lc=0, rc=0;
    for (int i=1; i<=60 && !c.fault; ++i) {
        base_loaded_command(&c, 150, 190, i*20000LL);
        lc -= 28; rc += 28; // about 200 mm/s on both wheels
        tick(&c, i*20000LL, lc, rc);
    }
    assert(c.fault==BASE_TRACKING && !c.active && c.pwm[0]==0 && c.pwm[1]==0);
    // Arc control asks for a wheel-speed difference while both wheels stay
    // in the observed moving-duty interval.
    base_init(&c, 0, 0, 0);
    assert(base_arc_command(&c, 35, 0));
    tick(&c, 20000, -10, 10);
    assert(c.arc_mode && c.pwm[0]>=.6f && c.pwm[0]<=.65f);
    assert(c.pwm[1]>c.pwm[0] && c.pwm[1]<=.65f);
    base_stop(&c);
    assert(base_arc_command(&c, -35, 40000));
    tick(&c, 60000, -20, 20);
    assert(c.pwm[0]>c.pwm[1] && c.pwm[0]<=.65f);
    base_stop(&c);
    assert(!base_arc_command(&c, 60, 80000));
    assert(!c.active);
    // A commanded arc that keeps going straight must latch a fault.
    base_init(&c, 0, 0, 0);
    lc=rc=0;
    for (int i=1; i<=70 && !c.fault; ++i) {
        base_arc_command(&c, 35, i*20000LL);
        lc -= 21; rc += 21;
        tick(&c, i*20000LL, lc, rc);
    }
    assert(c.fault==BASE_TRACKING && !c.active);
    // With a responsive fast wheel, the arc controller should hold the
    // requested speed difference and remain below the duty cap.
    base_init(&c, 0, 0, 0);
    float al=0, ar=0, apl=0, apr=0;
    for (int i=1; i<=80; ++i) {
        if (i%5==1) assert(base_arc_command(&c, 30, i*20000LL));
        al += .16f*(145 + 800*(c.pwm[0]-.6f)-al);
        ar += .16f*(145 + 800*(c.pwm[1]-.6f)-ar);
        apl += al*.02f/(210.0f/1468.0f);
        apr += ar*.02f/(210.0f/1468.0f);
        tick(&c, i*20000LL, -(int)lroundf(apl), (int)lroundf(apr));
        assert(c.fault==BASE_OK && c.pwm[0]>=.6f && c.pwm[0]<=.65f);
        assert(c.pwm[1]>=.6f && c.pwm[1]<=.65f);
    }
    assert(c.measured[1]-c.measured[0]>20);
    base_init(&c, 0, 0, 0);
    assert(base_duty_command(&c, .5f, .5f, 0));
    tick(&c, 20000, -20, 20);
    assert(c.duty_mode && c.pwm[0]==.5f && c.pwm[1]==.5f);
    tick(&c, 320000, -20, 20);
    assert(!c.active && !c.duty_mode && c.pwm[0]==0 && c.pwm[1]==0);
    assert(!base_duty_command(&c, .8f, .5f, 320000));
    base_init(&c, 0, 0, 0);
    assert(base_duty_command(&c, .5f, .5f, 0));
    for (int i=1; i<45; ++i) {
        base_duty_command(&c, .5f, .5f, i*20000LL);
        tick(&c, i*20000LL, 0, 0);
    }
    assert(c.fault==BASE_STALL && !c.active);
    // Tiny residual encoder noise must not disable the startup pulse.
    base_init(&c, 0, 0, 0);
    c.measured[0] = -0.5f;
    c.measured[1] = -0.5f;
    base_command(&c, 80, 80, 0);
    assert(!c.startup_done[0] && !c.startup_done[1]);
    tick(&c, 20000, 0, 0);
    assert(c.pwm[0] > .5f && c.pwm[0] < .55f);
    assert(c.pwm[1] > .5f && c.pwm[1] < .55f);

    // Startup pulse is bounded, keepalives must not rearm it, and stall
    // protection remains active when neither encoder moves.
    base_init(&c, 0, 0, 0);
    base_command(&c, 60, 60, 0);
    tick(&c, 20000, 0, 0);
    assert(c.pwm[0] > .5f && c.pwm[0] < .55f);
    assert(c.pwm[1] > .5f && c.pwm[1] < .55f);
    for (int i=2; i<=10; ++i) {
        base_command(&c, 60, 60, i*20000LL);
        tick(&c, i*20000LL, 0, 0);
    }
    assert(c.pwm[0] > .70f && c.pwm[0] <= .75f);
    assert(c.pwm[1] > .70f && c.pwm[1] <= .75f);
    base_init(&c, 0, 0, 0);
    base_command(&c, 60, 60, 0);
    tick(&c, 20000, 0, 0);
    for (int i=2; i<=80; ++i) {
        base_command(&c, 60, 60, i*20000LL);
        tick(&c, i*20000LL, 0, 0);
    }
    assert(c.fault==BASE_STALL && !c.active);
    base_stop(&c);
    assert(c.pwm[0]==0 && c.pwm[1]==0);
    base_init(&c, 0, 0, 0);
    base_command(&c, 60, 60, 0);
    tick(&c, 20000, 0, 0);
    tick(&c, 40000, -20, 20);
    assert(c.pwm[0]<.55f && c.pwm[1]<.55f);

    assert(c.pwm[0]>.50f && c.pwm[1]>.50f);
    base_command(&c, -60, -60, 40000);
    tick(&c, 60000, -30, 30);
    assert(c.pwm[0]<.55f && c.pwm[1]<.55f);

    // When a loaded wheel falls below target, do not keep lowering duty.
    base_init(&c, 0, 0, 0);
    base_command(&c, 100, 100, 0);
    c.startup_done[0] = c.startup_done[1] = true;
    c.setpoint[0] = c.setpoint[1] = 100;
    c.pwm[0] = c.pwm[1] = .45f;
    c.measured[0] = c.measured[1] = 100;
    tick(&c, 20000, 0, 0);
    assert(c.measured[0] < 100 && c.pwm[0] >= .45f);
    assert(c.measured[1] < 100 && c.pwm[1] >= .45f);

    // A loaded wheel that has accelerated far beyond a low speed command
    // must shed startup duty promptly instead of coasting at high speed.
    base_init(&c, 0, 0, 0);
    base_command(&c, 60, 60, 0);
    c.startup_done[0] = c.startup_done[1] = true;
    c.setpoint[0] = c.setpoint[1] = 60;
    c.pwm[0] = c.pwm[1] = .65f;
    c.measured[0] = c.measured[1] = 150;
    tick(&c, 20000, -21, 21);
    assert(c.pwm[0] < .62f && c.pwm[1] < .62f);
    assert(c.fault == BASE_OK);
    c.measured[0] = c.measured[1] = 10;
    c.pwm[0] = c.pwm[1] = .42f;
    c.startup_s[0] = c.startup_s[1] = .2f;
    for (int i=2; i<=6; ++i) {
        base_command(&c, 60, 60, i*20000LL);
        tick(&c, i*20000LL, -21, 21);
    }
    assert(c.pwm[0] > .5f && c.pwm[1] > .5f);

    base_init(&c, 0, 0, 0);
    tick(&c, 20000, 0, 0);
    assert(c.pwm[0] == 0 && c.pwm[1] == 0 && !c.active);
    assert(base_command(&c, 60, 60, 20000));
    tick(&c, 40000, 0, 0);
    assert(c.setpoint[0] > 0 && c.setpoint[0] <= 3.01f);
    assert(c.pwm[0] >= 0 && c.pwm[0] <= .75f);
    tick(&c, 320000, 0, 0);
    assert(!c.active && c.pwm[0] == 0 && c.pwm[1] == 0);

    base_init(&c, 0, 0, 0);
    assert(!base_command(&c, 101, 0, 0));
    assert(!base_command(&c, NAN, 0, 0));
    assert(!c.active);
    assert(base_command(&c, 60, 60, 0));
    base_stop(&c);
    tick(&c, 20000, 0, 0);
    assert(c.pwm[0] == 0 && c.pwm[1] == 0);

    base_init(&c, 0, 0, 0);
    for (int i=1; i<200; ++i) {
        base_command(&c, 100, 100, i*20000LL);
        tick(&c, i*20000LL, 0, 0);
        assert(fabsf(c.pwm[0]) <= .75f);
    }
    assert(c.fault == BASE_STALL);
    assert(!base_command(&c, 60, 60, 4000000));
    assert(c.pwm[0] == 0 && c.pwm[1] == 0);

    base_init(&c, 0, 0, 0);
    for (int i=1; i<40; ++i) {
        base_command(&c, 60, 60, i*20000LL);
        int edges = i>10 ? (i-10)*10 : 0;
        tick(&c, i*20000LL, edges, -edges);
    }
    assert(c.fault == BASE_DIRECTION);

    base_init(&c, 0, 0, 0);
    base_command(&c, 60, 60, 0);
    base_tick(&c, 20000, 0, 0, 20, 0);
    assert(c.fault == BASE_ENCODER);

    base_init(&c, 0, 0, 0);
    for (int i=1; i<25; ++i) {
        base_command(&c, 60, 60, i*20000LL);
        tick(&c, i*20000LL, -i*80, i*80);
    }
    assert(c.fault == BASE_OVERSPEED);

    base_init(&c, 0, 0, 0);
    base_command(&c, 60, 60, 0);
    tick(&c, 20000, -10, 10);
    assert(c.measured[0] > 0 && c.measured[1] > 0);
    base_command(&c, -60, -60, 20000);
    tick(&c, 40000, -20, 20);
    assert(c.pwm[0] >= 0 && c.pwm[1] >= 0);

    // Unequal simulated motors must converge to the same wheel-speed target.
    base_init(&c, 0, 0, 0);
    float vl=0, vr=0, pl=0, pr=0;
    for (int i=1; i<=300; ++i) {
        if (i%5==1) assert(base_command(&c, 60, 60, i*20000LL));
        vl += .12f*(200*c.pwm[0]-vl);
        vr += .12f*(160*c.pwm[1]-vr);
        pl += vl*.02f/(210.0f/1468.0f);
        pr += vr*.02f/(210.0f/1468.0f);
        tick(&c, i*20000LL, -(int)lroundf(pl), (int)lroundf(pr));
        assert(c.fault==0);
    }
    assert(fabsf(c.measured[0]-60)<5 && fabsf(c.measured[1]-60)<5);
    assert(c.pwm[1]>c.pwm[0]);

    base_init(&c, 0, 0, 0);
    base_command(&c, 60, 60, 0);
    tick(&c, 150000, 0, 0);
    assert(c.fault==BASE_TIMING && c.pwm[0]==0 && c.pwm[1]==0);

    float l=0, r=0;
    assert(parse_velocity("v 60 -40", &l, &r) && l==60 && r==-40);
    assert(!parse_velocity("v nan 0", &l, &r));
    assert(!parse_velocity("v 0 0 junk", &l, &r));
    assert(!parse_velocity("v 999 0", &l, &r));
    assert(!parse_velocity("v 60", &l, &r));
    assert(parse_loaded("b 160 180", &l, &r) && l==160 && r==180);
    assert(!parse_loaded("b 100 180", &l, &r));
    assert(!parse_loaded("b 160 180 junk", &l, &r));
    assert(parse_arc("a -35", &l) && l==-35);
    assert(!parse_arc("a 60", &l));
    assert(!parse_arc("a 20 junk", &l));
    // A short reverse-sync command must boost only the lagging wheel,
    // preserve signed reverse output, and stop when keepalives cease.
    base_init(&c, 0, 0, 0);
    assert(base_sync_command(&c, -1, 0));
    tick(&c, 20000, 0, 0);
    assert(c.sync_mode && c.pwm[0]<=-.6f && c.pwm[1]<=-.6f);
    tick(&c, 40000, 12, -24); // left reverse wheel is slower
    for (int i=3; i<=12; ++i) {
        if (i%5==1) assert(base_sync_command(&c, -1, i*20000LL));
        tick(&c, i*20000LL, 12+(i-2)*12, -24-(i-2)*12);
    }
    // Equal subsequent speeds must not erase the accumulated distance gap.
    assert(c.pwm[0]<-.61f && c.pwm[0]>=-.75f && c.pwm[1]==-.6f);
    tick(&c, 600000, 132, -144);
    assert(!c.active && c.pwm[0]==0 && c.pwm[1]==0);
    base_init(&c, 0, 0, 0);
    assert(!base_sync_command(&c, 0, 0));
    assert(!base_sync_command(&c, 2, 0));
    assert(parse_sync("y -1", &l) && l==-1);
    assert(!parse_sync("y 0", &l));
    assert(!parse_sync("y -1 junk", &l));
    base_init(&c, 0, 0, 0);
    int sl=0, sr=0;
    for (int i=1; i<=40 && !c.fault; ++i) {
        assert(base_sync_command(&c, -1, i*20000LL));
        sl += 20; sr -= 20;
        tick(&c, i*20000LL, sl, sr);
        assert(c.pwm[0]>=-.65f && c.pwm[1]>=-.65f);
    }
    assert(c.fault==BASE_OK);
    base_init(&c, 0, 0, 0);
    for (int i=1; i<=40 && !c.fault; ++i) {
        assert(base_sync_command(&c, -1, i*20000LL));
        tick(&c, i*20000LL, 0, 0);
    }
    assert(c.fault==BASE_STALL && !c.active && c.pwm[0]==0 && c.pwm[1]==0);
    // A positive turn is counterclockwise: left reverse, right forward.
    base_init(&c, 0, 0, 0);
    assert(base_turn_command(&c, 1, 0));
    tick(&c, 20000, 10, 10);
    assert(c.pwm[0]<0 && c.pwm[1]>0);
    tick(&c, 40000, 22, 30);
    assert(-c.pwm[0]>c.pwm[1]);
    assert(base_turn_command(&c, 1, 40000));
    assert(c.sync_distance[1]>c.sync_distance[0]);
    base_stop(&c);
    assert(base_turn_command(&c, -1, 60000));
    tick(&c, 80000, 12, 40);
    assert(c.pwm[0]>0 && c.pwm[1]<0);
    base_stop(&c);
    // Equal physical turn speeds must not look like opposite tracking errors.
    base_init(&c, 0, 0, 0);
    for (int i=1; i<=20 && !c.fault; ++i) {
        assert(base_turn_command(&c, 1, i*20000LL));
        tick(&c, i*20000LL, i*20, i*20);
    }
    assert(c.fault==BASE_OK && c.active);
    base_stop(&c);
    // Turn-only breakaway boost: a wheel with no encoder progress after
    // 150 ms gets a bounded pulse; a moving peer stays at its normal duty.
    base_init(&c, 0, 0, 0);
    for (int i=1; i<=9; ++i) {
        assert(base_turn_command(&c, 1, i*20000LL));
        tick(&c, i*20000LL, 0, i*6);
    }
    assert(c.pwm[0]<=-.74f && c.pwm[1]>=.6f && c.pwm[1]<.7f);
    // No movement after the boost must latch a fault before a long grind.
    base_init(&c, 0, 0, 0);
    for (int i=1; i<=20 && !c.fault; ++i) {
        assert(base_turn_command(&c, 1, i*20000LL));
        tick(&c, i*20000LL, 0, 0);
    }
    assert(c.fault==BASE_STALL && !c.active && c.pwm[0]==0 && c.pwm[1]==0);
    // A small initial twitch must not count as continued rotation.
    base_init(&c, 0, 0, 0);
    for (int i=1; i<=16 && !c.fault; ++i) {
        assert(base_turn_command(&c, 1, i*20000LL));
        tick(&c, i*20000LL, 6, 6);
    }
    assert(c.active && c.pwm[0]<=-.74f && c.pwm[1]>=.74f);
    for (int i=17; i<=21 && !c.fault; ++i) {
        assert(base_turn_command(&c, 1, i*20000LL));
        tick(&c, i*20000LL, 6, 6);
    }
    assert(c.fault==BASE_STALL && !c.active);
    // Even continuous keepalives cannot extend a diagnostic turn past 0.5 s.
    base_init(&c, 0, 0, 0);
    for (int i=1; i<=27 && !c.fault; ++i) {
        assert(base_turn_command(&c, 1, i*20000LL));
        tick(&c, i*20000LL, i*8, i*8);
    }
    assert(c.fault==BASE_TIMING && !c.active && c.pwm[0]==0 && c.pwm[1]==0);
    base_stop(&c);
    assert(!base_turn_command(&c, 0, 100000));
    assert(parse_turn("t 1", &l) && l==1);
    assert(parse_turn("t -1", &l) && l==-1);
    assert(!parse_turn("t 0", &l));
    assert(!parse_turn("t 1 junk", &l));
    loaded_plant_run(80, 80);
    loaded_plant_run(-80, -80);
    loaded_plant_run(-80, 80);
    base_init(&c, 0, 0, 0);
    for (int i=1; i<=30 && !c.fault; ++i) {
        if (i%5==1) assert(base_command(&c, -80, 80, i*20000LL));
        tick(&c, i*20000LL, 0, i*8);
    }
    assert(c.fault==BASE_STALL && !c.active && c.pwm[0]==0 && c.pwm[1]==0);
    puts("control and protocol tests passed");
}
