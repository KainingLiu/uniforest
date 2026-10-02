/* Real byte parser, dispatcher, state machines, and simulated peripherals. */
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
#ifdef HOST_ADDITIVE
#include "actions_extended.c"
#include "execution_session.c"
#endif
#include "protocol.c"

static unsigned servo_events, pump_events;
void Servo_HomeAll(void) { servo_events++; }
void Servo_SetAngle(uint8_t id,uint8_t angle) { (void)id; (void)angle; servo_events++; }
void Servo_SetAngleTenth(uint8_t id,uint16_t angle) { (void)id; (void)angle; servo_events++; }
void Suction_PumpOn(void) { pump_events++; }
void Suction_Release(void) { pump_events++; }
void Suction_AllOff(void) {}
float IMU_GetYaw(void) { return 0; }
float IMU_GetYawRate(void) { return 0; }
uint8_t IMU_IsReady(void) { return 0; }
void IMU_ResetYaw(void) {}
void IMU_Update(void) {}
uint16_t SBUS_GetChannel(uint8_t id) { return 1000+id; }

static unsigned serial_seq;
static void wire(uint8_t command,const uint8_t *data,uint8_t n,int corrupt)
{
    uint8_t bytes[86]={0xAA,command,(uint8_t)(n+5),(uint8_t)++serial_seq};
    if(n) memcpy(bytes+4,data,n);
    uint16_t crc=Protocol_CRC16(bytes+1,n+3);
    bytes[n+4]=crc&255; bytes[n+5]=(crc>>8)^(corrupt?1:0);
    for(unsigned i=0;i<n+6;i++) Protocol_ISR_FeedByte(bytes[i]);
    Protocol_RxPoll();
}
static void legacy_trace(void)
{
    uint8_t zero[31]={0}, speed[8]={0,10,255,246,0,20,255,236};
    wire(CMD_PING,0,0,0);
    wire(CMD_SET_TELEM_RATE,(uint8_t[]){0,50},2,0);
    wire(CMD_CHASSIS_SPEED,speed,8,0);
    wire(CMD_CHASSIS_TORQUE,zero,8,0);
    wire(CMD_CHASSIS_PID_SPEED,zero,21,0);
    wire(CMD_CHASSIS_PID_POS,zero,21,0);
    wire(CMD_CHASSIS_PID_RESET,zero,1,0);
    wire(CMD_SERVO_ANGLE,(uint8_t[]){1,90},2,0);
    wire(CMD_SERVO_ANGLE,(uint8_t[]){0,3,204},3,0);
    wire(CMD_SERVO_HOME,0,0,0);
    wire(CMD_SERVO_ANGLE_ALL,(uint8_t[]){90,90,63,117},4,0);
    wire(CMD_SUCTION,(uint8_t[]){1},1,0);
    wire(CMD_SUCTION,(uint8_t[]){2},1,0);
    wire(CMD_STEPPER_PARAMS,(uint8_t[]){1,144,0,60,1,144},6,0);
    wire(CMD_STEPPER_SET_POS,(uint8_t[]){0,0,0,0,123},5,0);
    wire(CMD_STEPPER_MOVE,(uint8_t[]){0,0,0,0,0,10},6,0);
    wire(CMD_STEPPER_STOP,zero,1,0);
    for(uint8_t cmd=CMD_STEPPER_MOVE_DUAL;cmd<=CMD_STEPPER_MOVE_DUAL3;cmd++) {
        if(cmd==CMD_STEPPER_SET_POS) continue;
        wire(cmd,zero,cmd==CMD_STEPPER_MOVE_DUAL?22:cmd==CMD_STEPPER_MOVE_DUAL2?27:31,0);
    }
    wire(CMD_ACTION_STATUS,0,0,0);
    wire(CMD_ACTION_START,(uint8_t[]){0,0,0,1,2,0},6,0);
    wire(CMD_SERVO_ANGLE,(uint8_t[]){1,90},2,0);
    wire(CMD_EMERGENCY_STOP,0,0,0);
    wire(CMD_ACTION_STATUS,0,0,0);
    wire(CMD_CHASSIS_SPEED,zero,7,0);
    wire(0x79,0,0,0);
    wire(CMD_PING,0,0,1);
    Protocol_SendTelemetry();
    for(unsigned i=0;i<host_uart_count;i++) {
        for(unsigned j=0;j<host_uart_lengths[i];j++) printf("%02x",host_uart_frames[i][j]);
        puts("");
    }
    printf("effects %u %u\n",servo_events,pump_events);
    for(unsigned i=0;i<4;i++) printf("target %d\n",g_motor[i].target_speed);
}
#ifdef HOST_ADDITIVE
static void open_session(uint32_t id)
{
    uint8_t data[6]; exec_put32(data,id); data[4]=0; data[5]=1;
    wire(CMD_EXEC_OPEN,data,6,0);
    assert(ExecutionSession_Id()==id);
}
static void wrapped(uint32_t id,uint8_t command,const uint8_t *data,uint8_t n)
{
    uint8_t payload[80]; exec_put32(payload,id); payload[4]=command;
    if(n)memcpy(payload+5,data,n);
    wire(CMD_EXEC_COMMAND,payload,n+5,0);
}
static void handoff_checks(void)
{
    wire(CMD_EXEC_CAPABILITIES,0,0,0);
    assert(host_uart_frames[host_uart_count-1][1]==TELEM_EXEC_CAPABILITIES);
    open_session(1);
    uint8_t action[]={0,0,0,42,3,0};
    wrapped(1,CMD_ACTION_START,action,6);
    assert(ActionsEx_IsBusy() && !Actions_IsBusy());
    wrapped(1,CMD_ACTION_STATUS,0,0);
    assert(host_uart_frames[host_uart_count-1][1]==TELEM_EXEC_ACTION);
    uint8_t speed[]={0,40,0,40,0,40,0,40};
    wrapped(1,CMD_CHASSIS_SPEED,speed,8);
    assert(g_motor[0].target_speed==40);
    /* A legacy heartbeat takes over immediately even within the lease. */
    wire(CMD_PING,0,0,0);
    assert(!ExecutionSession_Id() && !ActionsEx_IsBusy());
    assert(!Stepper_IsBusy(0) && !Stepper_IsBusy(1));
    for(unsigned i=0;i<4;i++)assert(!g_motor[i].target_speed);
    assert(Actions_GetStatus().state==ACTION_IDLE);
    wrapped(1,CMD_CHASSIS_SPEED,speed,8);
    assert(!g_motor[0].target_speed);
    /* Expiry is independent of base watchdog; a malformed extension cannot renew it. */
    open_session(2);
    wrapped(2,CMD_CHASSIS_SPEED,speed,8);
    host_tick+=190;
    wrapped(0xabcdef,CMD_PING,0,0);
    host_tick+=11; Protocol_RxPoll();
    assert(!ExecutionSession_Id() && !g_motor[0].target_speed);
    /* Raw legacy motion works again after expiry without resetting the MCU. */
    wire(CMD_CHASSIS_SPEED,speed,8,0);
    assert(g_motor[0].target_speed==40);
    /* Delayed shutdown from the old extension owner cannot stop legacy motion. */
    wire(CMD_EXEC_CLOSE,(uint8_t[]){0,0,0,2},4,0);
    assert(g_motor[0].target_speed==40);
    Motor3508_StopAllEnhanced();
    open_session(3);
    uint8_t pid[21]={0};
    float old_kp=g_motor[0].speed_pid.Kp;
    wrapped(3,CMD_CHASSIS_PID_SPEED,pid,21);
    assert(g_motor[0].speed_pid.Kp==old_kp);
    uint8_t id[4];exec_put32(id,3);
    wire(CMD_EXEC_CLOSE,id,4,0);
    assert(!ExecutionSession_Id());
    unsigned before=host_uart_count;
    wire(CMD_EXEC_CLOSE,id,4,0);
    assert(host_uart_count==before+1 && host_uart_frames[before][1]==TELEM_EXEC_SESSION);
    /* The same hardware can still execute old Grap2 with its old 5 cm event. */
    wire(CMD_ACTION_START,(uint8_t[]){0,0,0,87,2,0},6,0);
    assert(Actions_IsBusy());
    for(unsigned ms=0;ms<30000 && Actions_IsBusy();ms++) {
        for(unsigned t=0;t<100;t++)Stepper_Tick();
        host_tick++; Actions_Update();
        if(Actions_GetStatus().state==ACTION_CHASSIS_READY) {
            assert(!Stepper_FirstSegmentDone(1)); break;
        }
    }
    assert(Actions_GetStatus().state==ACTION_CHASSIS_READY);
    Actions_Abort(ACTION_CANCELLED);
    /* A much older OPEN cannot resurrect session 1 after sessions 2 and 3. */
    wire(CMD_EXEC_OPEN,(uint8_t[]){0,0,0,1,0,1},6,0);
    assert(!ExecutionSession_Id());
    assert(host_uart_frames[host_uart_count-1][5]==ACK_ERR_PARAM);
    /* The extension lease also handles uint32 tick wrap. */
    host_tick=0xfffffff0u;
    open_session(4);
    host_tick+=201u;Protocol_RxPoll();
    assert(!ExecutionSession_Id());
    puts("additive handoff passed");
}
static void extension_prelude(void)
{
    uint32_t trace_epoch=host_tick;
    open_session(1);
    uint8_t action[]={0,0,0,68,1,0};
    wrapped(1,CMD_ACTION_START,action,6);
    for(unsigned ms=0;ms<30000 && ActionsEx_IsBusy();ms++) {
        for(unsigned t=0;t<100;t++)Stepper_Tick();
        host_tick++;
        if(host_tick%50==0)wrapped(1,CMD_PING,0,0);
        ExecutionSession_Poll();
    }
    assert(ActionsEx_GetStatus().state==ACTION_DONE);
    wrapped(1,CMD_SET_TELEM_RATE,(uint8_t[]){0,100},2);
    wire(CMD_PING,0,0,0); /* legacy entry point triggers complete cleanup */
    assert(!ExecutionSession_Id() && Protocol_GetTelemetryRate()==0);
    assert(Actions_GetStatus().token==0 && Actions_GetStatus().state==ACTION_IDLE);
    /* Reset only the test clock/capture, not production control state. */
    host_tick=trace_epoch;
    serial_seq=host_uart_count=servo_events=pump_events=0;
}
#endif
int main(int argc,char **argv)
{
    Stepper_Init(); Motor3508_Init(); Protocol_Init();
#ifdef HOST_ADDITIVE
    if(argc>1 && !strcmp(argv[1],"handoff")) { handoff_checks(); return 0; }
    if(argc>1 && !strcmp(argv[1],"after")) extension_prelude();
#endif
    legacy_trace();
    return 0;
}
