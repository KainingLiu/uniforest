/* Include production implementations so tests observe real ISR/action logic. */
#include <assert.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#define __CAN_H__
#define __GPIO_H__
#ifdef _MSC_VER
/* Match GCC's packed protocol type while compiling with MSVC. */
#define __attribute__(attributes)
#pragma pack(push, 1)
#include "protocol.h"
#pragma pack(pop)
#endif
#include "stepper.c"
#include "actions.c"
#include "motor3508.c"

void Servo_HomeAll(void) {}
void Servo_SetAngle(uint8_t id, uint8_t angle) { (void)id; (void)angle; }
void Servo_SetAngleTenth(uint8_t id, uint16_t angle) { (void)id; (void)angle; }
void Suction_PumpOn(void) {}
void Suction_Release(void) {}
void Suction_AllOff(void) {}
float IMU_GetYaw(void) { return 0; }
uint8_t IMU_IsReady(void) { return 0; }
void IMU_ResetYaw(void) {}
void IMU_Update(void) {}

static void ticks(unsigned count)
{
    for (unsigned i = 0; i < count; ++i) Stepper_Tick();
}

static void pickup(uint8_t id)
{
    Stepper_Init();
    host_tick = 0;
    assert(Actions_Start(100u + id, id, 0) == ACK_OK);
    int saw_ready = 0, saw_ascent = 0;
    for (unsigned ms = 0; ms < 30000 && Actions_IsBusy(); ++ms) {
        ticks(100);
        host_tick++;
        /* Recognize the real ascent by its commanded direction and length. */
        if (g_stepper[V].dir == F && g_stepper[V].seg_steps ==
                (id == ACTION_GRAP3 ? S(9) : S(18.5))) {
            saw_ascent = 1;
        }
        Actions_Update();
        if (Actions_GetStatus().state == ACTION_CHASSIS_READY) {
            assert(saw_ascent);
            if (!saw_ready) {
                assert(Stepper_FirstSegmentDone(V));
                assert(Stepper_IsBusy(H) || Stepper_IsBusy(V));
                assert(Actions_Start(200u + id, id, 0) == ACK_ERR_BUSY);
            }
            saw_ready = 1;
        } else if (!saw_ready && saw_ascent &&
                   !Stepper_FirstSegmentDone(V)) {
            assert(Actions_GetStatus().state == ACTION_RUNNING);
        }
    }
    assert(saw_ready);
    assert(Actions_GetStatus().state == ACTION_DONE);
}

static void delayed_poll_after_peak(void)
{
    Stepper_Init();
    host_tick = 0;
    assert(Actions_Start(300, ACTION_GRAP3, 0) == ACK_OK);
    while (!(move_parallel && sequence[status.stage].op == WAIT_FIRST_SEGMENT)) {
        ticks(100); host_tick++; Actions_Update();
        assert(host_tick < 10000);
    }
    /* Let the interrupt finish the entire up/down move without a main poll.
     * The position is back at its origin, so signed-position gating loses it. */
    while (Stepper_IsBusy(H) || Stepper_IsBusy(V)) {
        ticks(100); host_tick++; assert(host_tick < 20000);
    }
    assert(Stepper_GetPosition(V) == move_origin[V]);
    assert(Stepper_FirstSegmentDone(V));
    Actions_Update();
    assert(Actions_GetStatus().state == ACTION_CHASSIS_READY);
    Actions_Abort(ACTION_CANCELLED);
    assert(!Stepper_FirstSegmentDone(V));
    ticks(1000); Actions_Update();
    assert(Actions_GetStatus().state == ACTION_CANCELLED);
}

static void timeout_and_latch_reset(void)
{
    Stepper_Init();
    Stepper_StartMove(V, F, 2, 400, 60, 1);
    ticks(500);
    assert(Stepper_FirstSegmentDone(V));
    Stepper_StartMove(V, R, 100, 400, 60, 1);
    assert(!Stepper_FirstSegmentDone(V));
    Stepper_Stop(V);
    assert(!Stepper_FirstSegmentDone(V));
    assert(Actions_Start(400, ACTION_GRAP1, 0) == ACK_OK);
    Actions_Update();
    host_tick += 30001;
    Actions_Update();
    assert(Actions_GetStatus().state == ACTION_TIMEOUT);
    assert(!Stepper_IsBusy(H) && !Stepper_IsBusy(V));
}

static void stop_remains_stopped(void)
{
    Motor3508_Init();
    int16_t targets[4] = {100, -200, 300, -400};
    Motor3508_SetAllSpeeds(targets);
    Motor3508_UpdateAllSpeedPID(0.001f);
    assert(g_torque[0] != 0);
    for (unsigned i = 0; i < 4; ++i) {
        g_motor[i].cumulative_pos = 1234 + i;
        g_motor[i].target_position = 9999;
        g_motor[i].pos_pid.integral = 42;
    }
    host_irq = 0;
    Motor3508_StopAll();
    assert(host_irq == 0 && host_can_irq == 1);
    for (unsigned i = 0; i < 4; ++i) {
        assert(g_motor[i].target_speed == 0);
        assert(g_motor[i].target_position == g_motor[i].cumulative_pos);
        assert(g_motor[i].pos_pid.integral == 0);
        assert(g_torque[i] == 0);
    }
    for (unsigned i = 0; i < 50; ++i) Motor3508_UpdateAllSpeedPID(0.001f);
    for (unsigned i = 0; i < 8; ++i) assert(host_last_can[i] == 0);
    host_irq = 1;
    Motor3508_StopAll();
    assert(host_irq == 1);
}

int main(void)
{
    pickup(ACTION_GRAP1);
    pickup(ACTION_GRAP2);
    pickup(ACTION_GRAP3);
    delayed_poll_after_peak();
    timeout_and_latch_reset();
    stop_remains_stopped();
    puts("firmware execution: lift milestones, delayed polling, cancellation, "
         "timeout and persistent stop passed");
    return 0;
}
