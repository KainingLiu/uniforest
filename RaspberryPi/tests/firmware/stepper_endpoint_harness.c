/* Exercise actual pulse/linked-move code, especially a trigger at leg completion. */
#include <assert.h>
#include <stdio.h>
#include "stepper.h"

static void replay(uint8_t lead, uint32_t offset) {
    uint8_t other = 1 - lead;
    Stepper_Stop(lead); Stepper_Stop(other);
    Stepper_StartMoveOverlap3(lead, CM_TO_STEPS(2), STEP_DIR_FORWARD,
                             CM_TO_STEPS(11), STEP_DIR_REVERSE,
                             other, CM_TO_STEPS(19.5), STEP_DIR_FORWARD,
                             offset, CM_TO_STEPS(17.5), 400, 60, 400);
    int saw_other = 0, saw_reverse = 0;
    int32_t previous = 0;
    unsigned tick;
    for (tick = 0; tick < 1000000; tick++) {
        Stepper_Tick();
        int32_t lpos = Stepper_GetPosition(lead);
        int32_t opos = Stepper_GetPosition(other);
        if (opos && !saw_other) {
            saw_other = 1;
            assert(lpos >= (int32_t)offset);
            if (offset == CM_TO_STEPS(2)) assert(lpos == (int32_t)offset);
        }
        if (lpos < previous && !saw_reverse) {
            saw_reverse = 1;
            assert(previous == (int32_t)CM_TO_STEPS(2));
            assert(opos >= (int32_t)CM_TO_STEPS(17.5));
        }
        previous = lpos;
        if (!Stepper_IsBusy(lead) && !Stepper_IsBusy(other)) break;
    }
    assert(tick < 1000000 && saw_other && saw_reverse);
    assert(Stepper_GetPosition(lead) == -(int32_t)CM_TO_STEPS(9));
    assert(Stepper_GetPosition(other) == (int32_t)CM_TO_STEPS(19.5));
}

int main(void) {
    Stepper_Init();
    for (uint8_t lead = 0; lead < 2; lead++) {
        replay(lead, 0);
        replay(lead, CM_TO_STEPS(1));
        replay(lead, CM_TO_STEPS(2));
    }
    puts("Real stepper: both axis orders and start/middle/endpoint triggers passed");
    return 0;
}
