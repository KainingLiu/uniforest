/* Execute the production action state machine with deterministic pulse progress.
 * This tests sequencing, not motor acceleration, travel limits or servo physics. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "actions.h"
#include "action_stub.h"

typedef struct {
    int32_t pos;
    uint32_t progress, length;
    uint8_t dir, busy, active;
} Axis;
static Axis axis[2];
static uint32_t now, lift_started, lift_finished;
static unsigned pumps, releases, homes, stops, moves;
static uint16_t angles[4];
static unsigned mode, lead, other, phase_axis, second_started, other_started;
static uint32_t other_trigger, reverse_trigger, second_length;
static uint8_t second_dir, variant;
static unsigned pulse_rate = 4;

uint32_t HAL_GetTick(void) { return now; }
uint8_t Stepper_IsBusy(uint8_t m) { return axis[m].busy; }
int32_t Stepper_GetPosition(uint8_t m) { return axis[m].pos; }
void Stepper_Stop(uint8_t m) { axis[m].busy = axis[m].active = 0; }
void Suction_AllOff(void) { stops++; }
void Servo_SetAngleTenth(uint8_t id, uint16_t angle) {
    if (id == 1 && angle == 500) {
        lift_started = now;
        if (variant == 1) {
            assert(axis[1].active && axis[1].dir == STEP_DIR_FORWARD);
            assert(axis[1].length == CM_TO_STEPS(20));
            assert(axis[1].progress == CM_TO_STEPS(15));
            assert(!axis[0].busy && axis[0].pos == (int32_t)CM_TO_STEPS(3.5));
            assert(angles[0] == 1022);
        }
        if (variant == 2 && releases == 0) {
            /* Arm starts at V = 1 cm; H waits for the full 2 cm rise. */
            assert(axis[1].active && axis[1].dir == STEP_DIR_FORWARD);
            assert(axis[1].length == CM_TO_STEPS(2) && axis[1].progress == CM_TO_STEPS(1));
            assert(!axis[0].active && axis[0].busy);
            assert(axis[0].pos == (int32_t)CM_TO_STEPS(3.5));
            assert(axis[0].progress == 0);
        }
    }
    if (id == 1 && (angle == 0 || angle == 40)) lift_finished = now + 140;
    if (variant == 2 && id == 0 && angle == 972) {
        assert(now >= lift_finished);
        assert(now - lift_started == 500);
    }
    angles[id] = angle;
}
void Servo_SetAngle(uint8_t id, uint8_t angle) {
    Servo_SetAngleTenth(id, angle * 10);
}
void Servo_HomeAll(void) {
    assert(!axis[0].busy && !axis[1].busy);
    homes++;
    angles[0] = 972; angles[1] = 900; angles[2] = 630; angles[3] = 1170;
}
void Suction_PumpOn(void) {
    pumps++;
    if (variant == 1) {
        assert(homes == 1 && angles[1] == 1000 && angles[0] == 1022);
        assert(angles[2] == 1300 && angles[3] == 500);
    }
    if (variant == 2 && pumps == 2) {
        assert(axis[0].pos <= (int32_t)CM_TO_STEPS(5));
        assert(axis[0].busy || axis[1].busy); /* pickup overlaps retraction */
    }
}
void Suction_Release(void) {
    assert(!axis[0].busy && !axis[1].busy);
    releases++;
    if (variant == 1) {
        assert(releases == 1 && angles[1] == 40 && angles[0] == 1022);
        assert(now >= lift_finished);
        assert(axis[0].pos == (int32_t)CM_TO_STEPS(22.5));
        assert(axis[1].pos == -(int32_t)CM_TO_STEPS(19));
    }
    if (variant == 2) {
        assert(angles[1] == 0 && angles[0] == 972);
        assert(axis[0].pos == (int32_t)CM_TO_STEPS(23));
        assert(axis[1].pos == -(int32_t)CM_TO_STEPS(releases == 1 ? 20 : 14));
    }
}

static void plan(unsigned m, uint32_t length, uint8_t dir, int active) {
    assert(length && m < 2);
    axis[m].length = length; axis[m].progress = 0;
    axis[m].dir = dir; axis[m].busy = 1; axis[m].active = active;
}
static void begin(uint16_t start, uint16_t target, uint16_t ramp) {
    assert(!axis[0].busy && !axis[1].busy);
    assert(start == 400 && target == 60 && ramp == 400);
    mode = second_started = other_started = 0;
    moves++;
}
void Stepper_StartMove(uint8_t m, uint8_t dir, uint32_t length,
                       uint16_t s, uint16_t t, uint16_t a) {
    begin(s, t, a); plan(m, length, dir, 1);
    if (variant == 1) {
        assert(m == 1 && dir == STEP_DIR_FORWARD && length == CM_TO_STEPS(20));
        assert(axis[0].pos == (int32_t)CM_TO_STEPS(3.5));
        assert(axis[1].pos == -(int32_t)CM_TO_STEPS(20));
    }
}
void Stepper_StartMoveOverlap(uint8_t m1, uint32_t n1, uint8_t d1,
                              uint8_t m2, uint32_t n2, uint8_t d2,
                              uint32_t off, uint16_t s, uint16_t t, uint16_t a) {
    begin(s, t, a); assert(m1 != m2 && off <= n1);
    if (variant == 1 && moves == 3) {
        /* Placement waits for the full rise AND the complete 500 ms lift. */
        assert(axis[1].pos == 0 && angles[1] == 40 && angles[0] == 1022);
        assert(now - lift_started >= 500);
        assert(m1 == 0 && n1 == CM_TO_STEPS(19) && d1 == STEP_DIR_FORWARD);
        assert(m2 == 1 && n2 == CM_TO_STEPS(19) && d2 == STEP_DIR_REVERSE);
        assert(off == CM_TO_STEPS(3));
    }
    mode = 1; lead = m1; other = m2; other_trigger = off;
    plan(m1, n1, d1, 1); plan(m2, n2, d2, off == 0);
    other_started = off == 0;
}
void Stepper_StartMoveOverlap2(uint8_t mc, uint32_t nc, uint8_t dc,
                               uint8_t mp, uint32_t n1, uint8_t d1,
                               uint32_t n2, uint8_t d2, uint32_t off,
                               uint16_t s, uint16_t t, uint16_t a) {
    begin(s, t, a); assert(mc != mp && off <= nc);
    mode = 2; lead = mc; phase_axis = mp;
    reverse_trigger = off; second_length = n2; second_dir = d2;
    plan(mc, nc, dc, 1); plan(mp, n1, d1, 1);
}
void Stepper_StartMoveOverlap3(uint8_t ml, uint32_t n1, uint8_t d1,
                               uint32_t n2, uint8_t d2, uint8_t mo,
                               uint32_t no, uint8_t dout, uint32_t off1,
                               uint32_t off2, uint16_t s, uint16_t t, uint16_t a) {
    begin(s, t, a); assert(ml != mo && off1 <= n1 && off2 <= no);
    mode = 3; lead = ml; other = mo; phase_axis = ml;
    other_trigger = off1; reverse_trigger = off2;
    second_length = n2; second_dir = d2;
    plan(ml, n1, d1, 1); plan(mo, no, dout, off1 == 0);
    other_started = off1 == 0;
}
static void advance(void) {
    for (unsigned m = 0; m < 2; m++) {
        Axis *x = &axis[m];
        if (!x->active) continue;
        uint32_t n = x->length - x->progress;
        if (n > pulse_rate) n = pulse_rate;
        x->pos += x->dir ? -(int32_t)n : (int32_t)n;
        x->progress += n;
        if (x->progress == x->length) {
            x->active = 0;
            if (!((mode == 2 || mode == 3) && m == phase_axis && !second_started))
                x->busy = 0;
        }
    }
    if ((mode == 1 || mode == 3) && !other_started &&
            axis[lead].progress >= other_trigger) {
        if (variant == 2 && releases == 0 && mode == 3) {
            assert(axis[1].pos == -(int32_t)CM_TO_STEPS(9));
            assert(axis[1].progress == CM_TO_STEPS(2));
        }
        axis[other].active = 1; other_started = 1;
    }
    if ((mode == 2 || mode == 3) && !second_started &&
            axis[phase_axis].progress == axis[phase_axis].length &&
            axis[mode == 2 ? lead : other].progress >= reverse_trigger) {
        second_started = 1; plan(phase_axis, second_length, second_dir, 1);
    }
}

int main(int argc, char **argv) {
    assert(argc >= 3);
    variant = (uint8_t)atoi(argv[1]);
    if (argc > 3) pulse_rate = (unsigned)atoi(argv[3]);
    uint8_t id = variant == 1 ? ACTION_BUILD1 : variant == 2 ? ACTION_BUILD2 : ACTION_BUILD3;
    assert(ACTION_BUILD == ACTION_BUILD3 && ACTION_BUILD3 == 4 && ACTION_BUILD2 == 5 && ACTION_BUILD1 == 6);
    assert(Actions_Start(0, id, 0) == ACK_ERR_PARAM);
    assert(Actions_Start(1, 7, 0) == ACK_ERR_PARAM);
    assert(Actions_Start(1, 255, 0) == ACK_ERR_PARAM);
    assert(Actions_Start(1, ACTION_BUILD2, 1) == ACK_ERR_PARAM);
    assert(Actions_Start(1, ACTION_BUILD3, 1) == ACK_ERR_PARAM);
    assert(Actions_Start(1, ACTION_BUILD1, 1) == ACK_ERR_PARAM);
    assert(Actions_Start(10, id, 0) == ACK_OK);
    int ready = 0, aborted = 0;
    for (now = 0; now < 119000 && Actions_IsBusy(); now++) {
        if (strcmp(argv[2], "timeout") &&
            (strcmp(argv[2], "stall_lift") || moves != 2)) advance();
        Actions_Update();
        if (!strcmp(argv[2], "cancel_lift") && angles[1] == 500) {
            assert(variant == 1 && axis[1].busy && !releases);
            Actions_Abort(ACTION_CANCELLED); aborted = 1;
        }
        if (now == 100) {
            unsigned before = moves;
            uint8_t stage = Actions_GetStatus().stage;
            assert(Actions_Start(10, id, 0) == ACK_OK); /* replay never restarts */
            assert(Actions_GetStatus().stage == stage && moves == before);
            assert(Actions_Start(11, id, 0) == ACK_ERR_BUSY);
        }
        if (Actions_GetStatus().state == ACTION_CHASSIS_READY && !ready) {
            ready = 1;
            assert(releases == variant && Actions_IsBusy());
            assert(axis[0].busy && axis[1].busy); /* tail still in flight */
            assert(Actions_Start(11, id, 0) == ACK_ERR_BUSY);
            if (!strcmp(argv[2], "cancel")) {
                Actions_Abort(ACTION_CANCELLED); aborted = 1;
            }
        }
    }
    if (!strcmp(argv[2], "timeout") || !strcmp(argv[2], "stall_lift")) {
        assert(Actions_GetStatus().state == ACTION_TIMEOUT && stops == 1);
        assert(!releases && !ready);
    } else if (aborted) {
        assert(Actions_GetStatus().state == ACTION_CANCELLED && stops == 1);
    } else {
        assert(Actions_GetStatus().state == ACTION_DONE && ready);
        assert(pumps == variant && releases == variant);
        assert(axis[0].pos == 0 && axis[1].pos == 0);
        assert(homes == (variant == 2 ? 1u : 2u));
        assert(angles[0] == 972 && angles[1] == 900 && angles[2] == 630 && angles[3] == 1170);
    }
    assert(!Actions_IsBusy() && !axis[0].busy && !axis[1].busy);
    unsigned before = moves;
    assert(Actions_Start(10, id, 0) == ACK_OK);
    Actions_Update();
    assert(moves == before); /* neither completion nor cancellation resumes on replay */
    printf("build%u %s: releases=%u, position=(%ld,%ld), state=%u\n",
           variant, argv[2], releases, (long)axis[0].pos, (long)axis[1].pos,
           Actions_GetStatus().state);
    return 0;
}
