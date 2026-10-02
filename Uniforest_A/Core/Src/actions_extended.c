#include "actions_extended.h"
#include "protocol.h"
#include "servo.h"
#include "stepper.h"
#include "suction.h"

/* Distances, tenths of degrees and dwell times migrated from Pi actions.py.
 * All waits run in the main loop; TIM7 and the communication watchdog stay live. */
enum { ex_END, ex_HOME, ex_HATCH, ex_ANGLE, ex_PUMP, ex_RELEASE, ex_WAIT, ex_MOVE, ex_DUAL, ex_DUAL2, ex_DUAL3,
       ex_DUAL3_ASYNC, ex_WAIT_PROGRESS, ex_JOIN, ex_DUAL_ASYNC, ex_CHASSIS_READY
       , ex_WAIT_FIRST_SEGMENT
};
typedef struct { uint8_t op; uint16_t p[11]; } ex_ActionStep;
#define EX_H STEPPER_HORIZ
#define EX_V STEPPER_VERT
#define EX_F STEP_DIR_FORWARD
#define EX_R STEP_DIR_REVERSE
#define EX_S(cm) CM_TO_STEPS(cm)
#define EX_W(ms) {ex_WAIT, {ms}}
#define EX_A(id, tenth) {ex_ANGLE, {id, tenth}}
#define EX_M(m, dir, cm) {ex_MOVE, {m, dir, EX_S(cm)}}
#define EX_D_OP(op, m1, cm1, d1, m2, cm2, d2, off) \
    {op, {m1, EX_S(cm1), d1, m2, EX_S(cm2), d2, EX_S(off)}}
#define EX_D(...) EX_D_OP(ex_DUAL, __VA_ARGS__)
#define EX_D2(mc, cc, dc, mp, c1, d1, c2, d2, off) \
    {ex_DUAL2, {mc, EX_S(cc), dc, mp, EX_S(c1), d1, EX_S(c2), d2, EX_S(off)}}
#define EX_D3_OP(op, ml, c1, d1, c2, d2, mo, co, dout, o1, o2) \
    {op, {ml, EX_S(c1), d1, EX_S(c2), d2, mo, EX_S(co), dout, EX_S(o1), EX_S(o2)}}
#define EX_D3(...) EX_D3_OP(ex_DUAL3, __VA_ARGS__)
#define EX_LIFT(target) EX_A(1,500), EX_W(90), EX_A(1,280), EX_W(90), EX_A(1,140), EX_W(90), \
    EX_A(1,60), EX_W(90), EX_A(1,target), EX_W(90), EX_W(50)
#define EX_DROP EX_A(2,670), EX_A(3,1130), {ex_RELEASE,{0}}

static const ex_ActionStep ex_grap1[] = {
    {ex_PUMP,{0}}, {ex_HOME,{0}}, {ex_HATCH,{0}},
    EX_D(EX_H,22,EX_F,EX_V,18.5,EX_R,5), EX_D_OP(ex_DUAL_ASYNC,EX_V,18.5,EX_F,EX_H,22,EX_R,5),
    {ex_WAIT_FIRST_SEGMENT,{EX_V}}, {ex_CHASSIS_READY,{0}}, {ex_JOIN,{0}},
    EX_DROP, {ex_HOME,{0}}, {ex_END,{0}}
};
static const ex_ActionStep ex_grap2[] = {
    {ex_PUMP,{0}}, {ex_HOME,{0}}, {ex_HATCH,{0}},
    EX_D(EX_H,27,EX_F,EX_V,18.5,EX_R,10), EX_D_OP(ex_DUAL_ASYNC,EX_V,18.5,EX_F,EX_H,27,EX_R,5),
    {ex_WAIT_FIRST_SEGMENT,{EX_V}}, {ex_CHASSIS_READY,{0}}, {ex_JOIN,{0}},
    EX_DROP, {ex_HOME,{0}}, {ex_END,{0}}
};
static const ex_ActionStep ex_grap3[] = {
    {ex_PUMP,{0}}, {ex_HOME,{0}}, EX_A(1,450), EX_A(0,522), {ex_HATCH,{0}},
    EX_D(EX_H,27,EX_F,EX_V,9,EX_R,17), EX_D3_OP(ex_DUAL3_ASYNC,EX_V,9,EX_F,9,EX_R,EX_H,21.5,EX_R,5,16),
    {ex_WAIT_FIRST_SEGMENT,{EX_V}}, {ex_CHASSIS_READY,{0}}, {ex_JOIN,{0}},
    EX_DROP, EX_D_OP(ex_DUAL_ASYNC,EX_V,9,EX_F,EX_H,5.5,EX_R,0),
    {ex_WAIT_PROGRESS,{EX_V,EX_S(5),EX_F}}, EX_A(1,900), {ex_JOIN,{0}},
    {ex_HOME,{0}}, {ex_END,{0}}
};
static const ex_ActionStep ex_build[] = {
    /* Only pickup-pose and EX_LIFT waits remain; no post-stepper settle. */
    {ex_HOME,{0}}, {ex_HATCH,{0}}, {ex_PUMP,{0}}, EX_M(EX_H,EX_F,3.5),
    EX_A(1,1000), EX_A(0,1022), EX_W(500), EX_A(0,952), EX_LIFT(30),
    EX_D(EX_H,19,EX_F,EX_V,19,EX_R,3), {ex_RELEASE,{0}},
    EX_D3(EX_V,10,EX_F,2,EX_R,EX_H,22.5,EX_R,3,19.5),
    {ex_PUMP,{0}}, EX_A(1,950), EX_A(0,1022), EX_W(500), EX_A(0,972), EX_LIFT(0),
    EX_D2(EX_H,23,EX_F,EX_V,2,EX_F,4.5,EX_R,21), {ex_RELEASE,{0}},
    /* Third pickup: keep EX_H retracting while EX_V descends at EX_H = 18 cm. */
    EX_D3_OP(ex_DUAL3_ASYNC,EX_V,4.5,EX_F,11.5,EX_R,EX_H,23,EX_R,1,18),
    {ex_WAIT_PROGRESS,{EX_H,EX_S(18),EX_R}}, EX_A(1,950), EX_A(0,1022), {ex_PUMP,{0}},
    {ex_JOIN,{0}},
    /* Lift with EX_V; start EX_H at EX_V = 15 cm, then descend at EX_H = 22 cm.
     * EX_V must finish its full 20.5 cm rise before reversing. */
    EX_D3_OP(ex_DUAL3_ASYNC,EX_V,20.5,EX_F,4,EX_R,EX_H,23,EX_F,15,22),
    EX_LIFT(0), EX_A(0,972), {ex_JOIN,{0}}, {ex_RELEASE,{0}},
    {ex_CHASSIS_READY,{0}},
    EX_D(EX_V,4,EX_F,EX_H,23,EX_R,0), EX_A(1,900),
    {ex_HOME,{0}}, {ex_END,{0}}
};

static ActionStatus_t ex_status;
static const ex_ActionStep *ex_sequence;
static uint8_t ex_waiting_move, ex_waiting_time, ex_test_settle;
static uint8_t ex_move_parallel;
static int32_t ex_move_origin[STEPPER_COUNT];
static uint32_t ex_wait_started, ex_wait_ms, ex_move_started, ex_action_started;

uint8_t ActionsEx_IsBusy(void)
{
    return ex_status.state == ACTION_RUNNING || ex_status.state == ACTION_CHASSIS_READY;
}
ActionStatus_t ActionsEx_GetStatus(void) { return ex_status; }

void ActionsEx_Reset(void)
{
    ex_status=(ActionStatus_t){0};
    ex_sequence=0;
    ex_waiting_move=ex_waiting_time=ex_test_settle=ex_move_parallel=0;
}

void ActionsEx_Abort(uint8_t state)
{
    if (ActionsEx_IsBusy() || ex_status.state == ACTION_DONE) ex_status.state = state;
    ex_waiting_move = ex_waiting_time = ex_test_settle = ex_move_parallel = 0;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    Stepper_Stop(EX_H);
    Stepper_Stop(EX_V);
    __set_PRIMASK(irq);
    Suction_AllOff();
}

uint8_t ActionsEx_Start(uint32_t token, uint8_t id, uint8_t flags)
{
    if (token == 0 || id < ACTION_GRAP1 || id > ACTION_BUILD || flags > 1 ||
        (id == ACTION_BUILD && flags)) return ACK_ERR_PARAM;
    /* EX_A replay can report the previous outcome but cannot restart a motion. */
    if (token == ex_status.token)
        return id == ex_status.id ? ACK_OK : ACK_ERR_PARAM;
    if (Actions_IsBusy() || ActionsEx_IsBusy() || Stepper_IsBusy(EX_H) || Stepper_IsBusy(EX_V))
        return ACK_ERR_BUSY;
    static const ex_ActionStep *const sequences[] = {ex_grap1, ex_grap2, ex_grap3, ex_build};
    ex_sequence = sequences[id - 1];
    ex_status = (ActionStatus_t){token, id, ACTION_RUNNING, 0};
    ex_waiting_move = ex_waiting_time = ex_move_parallel = 0;
    ex_test_settle = flags;
    ex_action_started = HAL_GetTick();
    return ACK_OK;
}

void ActionsEx_Update(void)
{
    if (!ActionsEx_IsBusy()) return;
    uint32_t now = HAL_GetTick();
    if (now - ex_action_started >= 120000u) {
        ActionsEx_Abort(ACTION_TIMEOUT);
        return;
    }
    if (ex_waiting_move) {
        if (now - ex_move_started >= 30000u) {
            ActionsEx_Abort(ACTION_TIMEOUT);
            return;
        }
        if (Stepper_IsBusy(EX_H) || Stepper_IsBusy(EX_V)) {
            if (!ex_move_parallel) return;
        } else {
            ex_waiting_move = ex_move_parallel = 0;
        }
        /* Continue immediately once both axes finish; no fixed settle delay. */
    }
    if (ex_waiting_time) {
        if (now - ex_wait_started < ex_wait_ms) return;
        ex_waiting_time = 0;
    }
    /* Consume instantaneous operations together, preserving servo overlap. */
    while (ActionsEx_IsBusy()) {
        const ex_ActionStep *s = &ex_sequence[ex_status.stage];
        const uint16_t *p = s->p;
        if (s->op == ex_WAIT_FIRST_SEGMENT && !Stepper_FirstSegmentDone(p[0])) {
            if (!ex_waiting_move) ActionsEx_Abort(ACTION_TIMEOUT);
            return;
        }
        if (s->op == ex_WAIT_PROGRESS) {
            /* Read pulse-count progress in the main loop, never block the ISR. */
            int64_t progress = (int64_t)Stepper_GetPosition(p[0]) - ex_move_origin[p[0]];
            if (p[2] == EX_R) progress = -progress;
            if (progress < p[1]) {
                if (!ex_waiting_move) ActionsEx_Abort(ACTION_TIMEOUT);
                return;
            }
        }
        if ((s->op == ex_JOIN || s->op == ex_END) && ex_waiting_move) return;
        if (s->op == ex_END) {
            if (ex_test_settle) {
                ex_test_settle = 0;
                ex_waiting_time = 1;
                ex_wait_started = now;
                ex_wait_ms = 1000;
            } else ex_status.state = ACTION_DONE;
            return;
        }
        ex_status.stage++;
        switch (s->op) {
        case ex_WAIT_PROGRESS:
        case ex_WAIT_FIRST_SEGMENT:
        case ex_JOIN: break;
        case ex_CHASSIS_READY: ex_status.state = ACTION_CHASSIS_READY; break;
        case ex_HOME: Servo_HomeAll(); break;
        case ex_HATCH:
            Servo_SetAngle(2, 130); Servo_SetAngle(3, 50); break;
        case ex_ANGLE: Servo_SetAngleTenth(p[0], p[1]); break;
        case ex_PUMP: Suction_PumpOn(); break;
        case ex_RELEASE: Suction_Release(); break;
        case ex_WAIT:
            ex_waiting_time = 1; ex_wait_started = now; ex_wait_ms = p[0]; return;
        default: {
            /* EX_A new move may not replace an unfinished parallel move. */
            if (ex_waiting_move) { ActionsEx_Abort(ACTION_CANCELLED); return; }
            /* TIM7 must not observe a partially initialized linked move. */
            uint32_t irq = __get_PRIMASK();
            __disable_irq();
            ex_move_origin[EX_H] = Stepper_GetPosition(EX_H);
            ex_move_origin[EX_V] = Stepper_GetPosition(EX_V);
            switch (s->op) {
            case ex_MOVE:
                Stepper_StartMove(p[0], p[1], p[2], 400, 60, 400); break;
            case ex_DUAL: case ex_DUAL_ASYNC:
                Stepper_StartMoveOverlap(p[0],p[1],p[2],p[3],p[4],p[5],p[6],
                                        400,60,400); break;
            case ex_DUAL2:
                Stepper_StartMoveOverlap2(p[0],p[1],p[2],p[3],p[4],p[5],p[6],
                                         p[7],p[8],400,60,400); break;
            case ex_DUAL3: case ex_DUAL3_ASYNC:
                Stepper_StartMoveOverlap3(p[0],p[1],p[2],p[3],p[4],p[5],p[6],
                                         p[7],p[8],p[9],400,60,400); break;
            }
            __set_PRIMASK(irq);
            ex_waiting_move = 1; ex_move_started = now;
            ex_move_parallel = (s->op == ex_DUAL_ASYNC || s->op == ex_DUAL3_ASYNC);
            if (!ex_move_parallel) return;
            break;
        }
        }
    }
}
