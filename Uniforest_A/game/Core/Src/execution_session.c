/* Additive, volatile execution ownership. Nothing here changes a legacy run. */
#include "execution_session.h"
#include "actions_extended.h"
#include "motor3508.h"
#include "stepper.h"
#include <string.h>

static uint32_t exec_id, exec_retired_id, exec_last_seen;
static uint32_t exec_next_id=1; /* Never reused within one MCU boot. */
static uint16_t exec_saved_rate;

static uint32_t exec_u32(const uint8_t *p)
{
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static void exec_put32(uint8_t *p, uint32_t v)
{
    p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v;
}
static void exec_reply_session(uint8_t seq, uint32_t id, uint8_t active)
{
    uint8_t data[5]; exec_put32(data,id); data[4]=active;
    Protocol_SendFrame(TELEM_EXEC_SESSION,seq,data,sizeof(data));
}
static void exec_stop(void)
{
    if (!exec_id) return;
    Motor3508_StopAllEnhanced();
    ActionsEx_Abort(ACTION_CANCELLED);
    Protocol_RestoreTelemetryRate(exec_saved_rate);
    exec_retired_id=exec_id;
    exec_id=0;
}
uint32_t ExecutionSession_Id(void) { return exec_id; }
void ExecutionSession_LegacyTakeover(void) { exec_stop(); }
void ExecutionSession_Poll(void)
{
    if (!exec_id) return;
    if ((uint32_t)(HAL_GetTick()-exec_last_seen)>EXEC_LEASE_MS) {
        exec_stop();
        return;
    }
    ActionsEx_Update();
    ActionStatus_t s=ActionsEx_GetStatus();
    if (s.state==ACTION_CANCELLED || s.state==ACTION_TIMEOUT || s.state==ACTION_REJECTED)
        exec_stop();
}
static void exec_action_status(uint8_t seq, ActionStatus_t s)
{
    uint8_t data[15];
    exec_put32(data,exec_id); exec_put32(data+4,s.token);
    data[8]=s.id; data[9]=s.state; data[10]=s.stage;
    exec_put32(data+11,HAL_GetTick());
    Protocol_SendFrame(TELEM_EXEC_ACTION,seq,data,sizeof(data));
}
static uint8_t exec_valid_inner(uint8_t cmd, uint8_t n)
{
    /* Persistent tuning/position-reset commands stay in the legacy debug
     * interface. A match session cannot leave changed tuning for its successor. */
    switch(cmd) {
    case CMD_PING: case CMD_EMERGENCY_STOP: case CMD_SERVO_HOME:
    case CMD_ACTION_STATUS: return n==0;
    case CMD_CHASSIS_SPEED: return n==8;
    case CMD_SERVO_ANGLE: return n==2 || n==3;
    case CMD_SERVO_ANGLE_ALL: return n==4;
    case CMD_SUCTION: case CMD_STEPPER_STOP: return n==1;
    case CMD_STEPPER_MOVE: case CMD_ACTION_START: return n==6;
    case CMD_STEPPER_MOVE_DUAL: return n==22;
    case CMD_STEPPER_MOVE_DUAL2: return n==27;
    case CMD_STEPPER_MOVE_DUAL3: return n==31;
    case CMD_SET_TELEM_RATE: return n==2;
    default: return 0;
    }
}
uint8_t ExecutionSession_Handle(const ProtoFrame_t *f)
{
    const uint8_t *d=f->data;
    if (f->cmd==CMD_EXEC_CAPABILITIES) {
        if (f->data_len) Protocol_SendAck(f->cmd,f->seq,ACK_ERR_PARAM);
        else {
            uint8_t data[12]={0,EXEC_VERSION,0,0,0,EXEC_CAPABILITIES,0,EXEC_LEASE_MS};
            exec_put32(data+8,exec_next_id);
            Protocol_SendFrame(TELEM_EXEC_CAPABILITIES,f->seq,data,sizeof(data));
        }
        return 1;
    }
    if (f->cmd==CMD_EXEC_OPEN) {
        if (f->data_len!=6 || !exec_u32(d) || d[4]!=0 || d[5]!=EXEC_VERSION ||
            exec_u32(d)==exec_retired_id) {
            Protocol_SendAck(f->cmd,f->seq,ACK_ERR_PARAM); return 1;
        }
        uint32_t id=exec_u32(d);
        if (exec_id==id) { exec_reply_session(f->seq,id,1); return 1; }
        if (!exec_next_id || id!=exec_next_id) {
            Protocol_SendAck(f->cmd,f->seq,ACK_ERR_PARAM); return 1;
        }
        if (exec_id || Actions_IsBusy() || Stepper_IsBusy(0) || Stepper_IsBusy(1) ||
            !Motor3508_ExtensionCanStart()) {
            Protocol_SendAck(f->cmd,f->seq,ACK_ERR_BUSY); return 1;
        }
        exec_saved_rate=Protocol_GetTelemetryRate();
        Motor3508_StopAllEnhanced();
        /* Discard only the extension engine's latched previous outcome. */
        ActionsEx_Reset();
        exec_id=id; exec_next_id++; exec_last_seen=HAL_GetTick();
        exec_reply_session(f->seq,id,1); return 1;
    }
    if (f->cmd==CMD_EXEC_CLOSE) {
        if (f->data_len!=4 || !exec_u32(d) ||
            (exec_u32(d)!=exec_id && !(exec_id==0 && exec_u32(d)==exec_retired_id))) {
            Protocol_SendAck(f->cmd,f->seq,ACK_ERR_PARAM); return 1;
        }
        uint32_t id=exec_u32(d); exec_stop();
        exec_reply_session(f->seq,id,0); return 1;
    }
    if (f->cmd!=CMD_EXEC_COMMAND) return 0;
    if (f->data_len<5 || !exec_id || exec_u32(d)!=exec_id ||
        !exec_valid_inner(d[4],f->data_len-5)) {
        Protocol_SendAck(f->cmd,f->seq,ACK_ERR_PARAM); return 1;
    }
    ProtoFrame_t inner={0};
    inner.cmd=d[4]; inner.seq=f->seq; inner.data_len=f->data_len-5;
    inner.len=inner.data_len+5;
    memcpy(inner.data,d+5,inner.data_len);
    exec_last_seen=HAL_GetTick();
    if (inner.cmd==CMD_EMERGENCY_STOP) {
        exec_stop(); Protocol_SendAck(inner.cmd,f->seq,ACK_OK); return 1;
    }
    if (inner.cmd==CMD_ACTION_START) {
        uint32_t token=exec_u32(inner.data);
        uint8_t result=ActionsEx_Start(token,inner.data[4],inner.data[5]);
        ActionStatus_t s=ActionsEx_GetStatus();
        if (result!=ACK_OK) s=(ActionStatus_t){token,inner.data[4],ACTION_REJECTED,result};
        exec_action_status(f->seq,s);
        Protocol_SendAck(inner.cmd,f->seq,result); return 1;
    }
    if (inner.cmd==CMD_ACTION_STATUS) {
        exec_action_status(f->seq,ActionsEx_GetStatus()); return 1;
    }
    if (ActionsEx_IsBusy()) {
        if (inner.cmd==CMD_STEPPER_STOP && inner.data[0]<STEPPER_COUNT) {
            exec_stop(); Protocol_SendAck(inner.cmd,f->seq,ACK_OK); return 1;
        }
        if ((inner.cmd>=CMD_SERVO_ANGLE && inner.cmd<=CMD_SUCTION) ||
            (inner.cmd>=CMD_STEPPER_MOVE && inner.cmd<=CMD_STEPPER_MOVE_DUAL3)) {
            Protocol_SendAck(inner.cmd,f->seq,ACK_ERR_BUSY); return 1;
        }
    }
    Protocol_DispatchLegacy(&inner);
    return 1;
}
