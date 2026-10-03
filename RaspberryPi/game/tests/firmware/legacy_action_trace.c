/* The same input script is compiled with real fixed-main and additive sources. */
#include <assert.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#define __CAN_H__
#define __GPIO_H__
#ifdef _MSC_VER
#define __attribute__(attributes)
#pragma pack(push, 1)
#include "protocol.h"
#pragma pack(pop)
#endif
#include "stepper.c"
#include "actions.c"
#include "motor3508.c"
void Servo_HomeAll(void) { printf("home %u\n",host_tick); }
void Servo_SetAngle(uint8_t id,uint8_t a) { printf("servo %u %u %u\n",host_tick,id,a); }
void Servo_SetAngleTenth(uint8_t id,uint16_t a) { printf("servo10 %u %u %u\n",host_tick,id,a); }
void Suction_PumpOn(void) { printf("pump %u\n",host_tick); }
void Suction_Release(void) { printf("release %u\n",host_tick); }
void Suction_AllOff(void) { printf("off %u\n",host_tick); }
float IMU_GetYaw(void) { return 0; }
uint8_t IMU_IsReady(void) { return 0; }
void IMU_ResetYaw(void) {}
void IMU_Update(void) {}
static uint32_t token=20;
static void run(uint8_t id,uint8_t test,unsigned period)
{
    Stepper_Init();host_tick=0;
    uint32_t current=token++;
    assert(Actions_Start(current,id,test)==ACK_OK);
    assert(Actions_Start(current,id,test)==ACK_OK);
    assert(Actions_Start(token+100,id,test)==ACK_ERR_BUSY);
    uint8_t last_stage=255,last_state=255;
    for(unsigned ms=0;ms<30000 && Actions_IsBusy();ms++) {
        for(unsigned t=0;t<100;t++)Stepper_Tick();
        host_tick++;
        if(host_tick%period)continue;
        Actions_Update();ActionStatus_t s=Actions_GetStatus();
        if(s.stage!=last_stage || s.state!=last_state) {
            printf("state %u %u %u %u %u %ld %ld\n",id,test,host_tick,s.state,s.stage,
                (long)Stepper_GetPosition(0),(long)Stepper_GetPosition(1));
            last_stage=s.stage;last_state=s.state;
        }
    }
    assert(Actions_GetStatus().state==ACTION_DONE);
    assert(Actions_Start(current,id,test)==ACK_OK && !Actions_IsBusy());
}
int main(void)
{
    for(unsigned period=1;period<=50;period+=49)
        for(uint8_t id=1;id<=4;id++) {
            run(id,0,period);
            if(id<4)run(id,1,period);
        }
    assert(Actions_Start(token++,1,0)==ACK_OK);
    Actions_Update();host_tick+=30001;Actions_Update();
    assert(Actions_GetStatus().state==ACTION_TIMEOUT);
    assert(Actions_Start(token++,3,0)==ACK_OK);
    Actions_Update();Actions_Abort(ACTION_CANCELLED);Actions_Update();
    assert(Actions_GetStatus().state==ACTION_CANCELLED);
    Motor3508_Init();int16_t targets[4]={100,-200,300,-400};
    Motor3508_SetAllSpeeds(targets);
    for(unsigned i=0;i<4;i++) {g_motor[i].target_position=999;g_motor[i].pos_pid.integral=42;}
    Motor3508_StopAll();
    for(unsigned i=0;i<4;i++) {
        assert(g_motor[i].target_speed==targets[i]);
        assert(g_motor[i].target_position==999 && g_motor[i].pos_pid.integral==42);
    }
    Motor3508_UpdateAllSpeedPID(.001f);
    for(unsigned i=0;i<4;i++)printf("stop-next-torque %d\n",g_torque[i]);
    puts("legacy action trace complete");
    return 0;
}
