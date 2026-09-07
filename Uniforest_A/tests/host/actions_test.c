#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "actions.h"
#include "protocol.h"
#include "servo.h"
#include "stepper.h"
#include "suction.h"

static uint32_t tick, busy_until;
static int busy, stuck, emit, outputs;
uint32_t HAL_GetTick(void) { return tick; }
static void event(const char *name, const uint32_t *p, int n)
{
    outputs++;
    if (!emit) return;
    printf("[%u,\"%s\"", tick, name);
    for (int i = 0; i < n; i++) printf(",%u", p[i]);
    puts("]");
}
void Servo_HomeAll(void) { event("home", NULL, 0); }
void Servo_SetAngleTenth(uint8_t id, uint16_t tenth)
{ uint32_t p[] = {id,tenth}; event("angle", p, 2); }
void Servo_SetAngle(uint8_t id, uint8_t angle)
{ Servo_SetAngleTenth(id, angle * 10); }
void Suction_PumpOn(void) { event("pump", NULL, 0); }
void Suction_Release(void) { event("release", NULL, 0); }
void Suction_AllOff(void) { event("off", NULL, 0); }
uint8_t Stepper_IsBusy(uint8_t motor)
{ (void)motor; return busy && (stuck || (int32_t)(busy_until - tick) > 0); }
void Stepper_Stop(uint8_t motor) { (void)motor; busy = 0; }
static void move(const char *name, uint32_t *p, int n)
{ assert(!Stepper_IsBusy(0)); event(name,p,n); busy = 1; busy_until = tick + 10; }
void Stepper_StartMove(uint8_t m, uint8_t d, uint32_t s,
                      uint16_t start, uint16_t target, uint16_t accel)
{ uint32_t p[]={m,d,s,start,target,accel}; move("move",p,6); }
void Stepper_StartMoveOverlap(uint8_t m1,uint32_t s1,uint8_t d1,
    uint8_t m2,uint32_t s2,uint8_t d2,uint32_t off,
    uint16_t start,uint16_t target,uint16_t accel)
{ uint32_t p[]={m1,s1,d1,m2,s2,d2,off,start,target,accel}; move("dual",p,10); }
void Stepper_StartMoveOverlap2(uint8_t mc,uint32_t sc,uint8_t dc,
    uint8_t mp,uint32_t s1,uint8_t d1,uint32_t s2,uint8_t d2,uint32_t off,
    uint16_t start,uint16_t target,uint16_t accel)
{ uint32_t p[]={mc,sc,dc,mp,s1,d1,s2,d2,off,start,target,accel}; move("dual2",p,12); }
void Stepper_StartMoveOverlap3(uint8_t ml,uint32_t s1,uint8_t d1,
    uint32_t s2,uint8_t d2,uint8_t mo,uint32_t so,uint8_t dout,uint32_t o1,uint32_t o2,
    uint16_t start,uint16_t target,uint16_t accel)
{ uint32_t p[]={ml,s1,d1,s2,d2,mo,so,dout,o1,o2,start,target,accel}; move("dual3",p,13); }

int main(int argc, char **argv)
{
    if (argc > 1) {
        emit = 1;
        uint8_t id = (uint8_t)atoi(argv[1]);
        assert(Actions_Start(1,id,argc > 2) == ACK_OK);
        for (tick=0; tick<120001; tick++) {
            Actions_Update();
            if (!Actions_IsBusy()) break;
        }
        assert(Actions_GetStatus().state == ACTION_DONE);
        event("done",NULL,0);
        return 0;
    }
    assert(Actions_Start(0,1,0) == ACK_ERR_PARAM);
    assert(Actions_Start(1,0,0) == ACK_ERR_PARAM);
    assert(Actions_Start(1,4,1) == ACK_ERR_PARAM);
    busy=stuck=1;
    assert(Actions_Start(1,1,0) == ACK_ERR_BUSY);
    busy=stuck=0;
    /* Cancel at every reachable stage, including servo waits and linked moves. */
    uint32_t token=1;
    for (uint8_t id=1; id<=4; id++) {
        for (uint8_t stage=0; stage<100; stage++) {
            assert(Actions_Start(token++,id,0) == ACK_OK);
            assert(Actions_Start(token,1,0) == ACK_ERR_BUSY);
            while (Actions_IsBusy() && Actions_GetStatus().stage < stage) {
                Actions_Update(); tick++;
            }
            int completed = !Actions_IsBusy();
            Actions_Abort(ACTION_CANCELLED);
            int count=outputs;
            for (int i=0;i<100;i++) { tick++; Actions_Update(); }
            assert(outputs == count && !Stepper_IsBusy(0));
            assert(Actions_Start(token-1,id,0) == ACK_OK);
            assert(!Actions_IsBusy());
            if (completed) break;
            assert(Actions_GetStatus().state == ACTION_CANCELLED);
        }
    }
    assert(Actions_Start(token++,1,0) == ACK_OK);
    Actions_Update(); stuck=1;
    tick+=30000; Actions_Update();
    assert(Actions_GetStatus().state == ACTION_TIMEOUT && !busy);
    stuck=0;
    tick=UINT32_MAX-100;
    assert(Actions_Start(token++,1,0) == ACK_OK);
    for (int i=0;i<5000 && Actions_IsBusy();i++) { Actions_Update(); tick++; }
    assert(Actions_GetStatus().state == ACTION_DONE);
    assert(Actions_Start(token++,4,0) == ACK_OK);
    tick+=120000; Actions_Update();
    assert(Actions_GetStatus().state == ACTION_TIMEOUT);
    puts("actions safety tests passed");
    return 0;
}
