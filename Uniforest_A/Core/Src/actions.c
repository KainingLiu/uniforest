#include "actions.h"
#include "protocol.h"
#include "servo.h"
#include "stepper.h"
#include "suction.h"

/* Distances, tenths of degrees and dwell times migrated from Pi actions.py.
 * All waits run in the main loop; TIM7 and the communication watchdog stay live. */
enum { END, HOME, HATCH, ANGLE, PUMP, RELEASE, WAIT, MOVE, DUAL, DUAL2, DUAL3,
       DUAL3_ASYNC, WAIT_PROGRESS, JOIN, DUAL_ASYNC, CHASSIS_READY };
typedef struct { uint8_t op; uint16_t p[11]; } ActionStep;
#define H STEPPER_HORIZ
#define V STEPPER_VERT
#define F STEP_DIR_FORWARD
#define R STEP_DIR_REVERSE
#define S(cm) CM_TO_STEPS(cm)
#define W(ms) {WAIT, {ms}}
#define A(id, tenth) {ANGLE, {id, tenth}}
#define M(m, dir, cm) {MOVE, {m, dir, S(cm)}}
#define D_OP(op, m1, cm1, d1, m2, cm2, d2, off) \
    {op, {m1, S(cm1), d1, m2, S(cm2), d2, S(off)}}
#define D(...) D_OP(DUAL, __VA_ARGS__)
#define D2(mc, cc, dc, mp, c1, d1, c2, d2, off) \
    {DUAL2, {mc, S(cc), dc, mp, S(c1), d1, S(c2), d2, S(off)}}
#define D3_OP(op, ml, c1, d1, c2, d2, mo, co, dout, o1, o2) \
    {op, {ml, S(c1), d1, S(c2), d2, mo, S(co), dout, S(o1), S(o2)}}
#define D3(...) D3_OP(DUAL3, __VA_ARGS__)
#define LIFT(target) A(1,500), W(90), A(1,280), W(90), A(1,140), W(90), \
    A(1,60), W(90), A(1,target), W(90), W(50)
#define DROP A(2,670), A(3,1130), {RELEASE,{0}}

static const ActionStep grap1[] = {
    {PUMP,{0}}, {HOME,{0}}, {HATCH,{0}},
    D(H,22,F,V,18,R,5), D(V,18,F,H,22,R,5),
    DROP, {HOME,{0}}, {END,{0}}
};
static const ActionStep grap2[] = {
    {PUMP,{0}}, {HOME,{0}}, {HATCH,{0}},
    D(H,27,F,V,18,R,10), D_OP(DUAL_ASYNC,V,18,F,H,27,R,5),
    {WAIT_PROGRESS,{V,S(5),F}}, {CHASSIS_READY,{0}}, {JOIN,{0}},
    DROP, {HOME,{0}}, {END,{0}}
};
static const ActionStep grap3[] = {
    {PUMP,{0}}, {HOME,{0}}, A(1,450), A(0,522), {HATCH,{0}},
    D(H,27,F,V,9,R,17), D3(V,9,F,9,R,H,22,R,5,14),
    DROP, D(V,9,F,H,5,R,0), {HOME,{0}}, {END,{0}}
};
static const ActionStep build[] = {
    /* Only pickup-pose and LIFT waits remain; no post-stepper settle. */
    {HOME,{0}}, {HATCH,{0}}, {PUMP,{0}}, M(H,F,4),
    A(1,1000), A(0,1022), W(500), A(0,952), LIFT(30),
    D(H,19,F,V,19,R,3), {RELEASE,{0}},
    D3(V,10,F,2,R,H,23,R,3,20),
    {PUMP,{0}}, A(1,950), A(0,1022), W(500), A(0,972), LIFT(0),
    D2(H,23,F,V,2,F,5,R,21), {RELEASE,{0}},
    /* Third pickup: keep H retracting while V descends at H = 18 cm. */
    D3_OP(DUAL3_ASYNC,V,5,F,11.5,R,H,23,R,1,18),
    {WAIT_PROGRESS,{H,S(18),R}}, A(1,950), A(0,1022), {PUMP,{0}},
    {JOIN,{0}},
    /* Lift with V; start H at V = 15 cm, then descend at H = 22 cm.
     * V must finish its full 20.5 cm rise before reversing. */
    D3_OP(DUAL3_ASYNC,V,20.5,F,4,R,H,23,F,15,22),
    LIFT(0), A(0,972), {JOIN,{0}}, {RELEASE,{0}},
    {CHASSIS_READY,{0}},
    D(V,4,F,H,23,R,0), A(1,900),
    {HOME,{0}}, {END,{0}}
};

static ActionStatus_t status;
static const ActionStep *sequence;
static uint8_t waiting_move, waiting_time, test_settle;
static uint8_t move_parallel;
static int32_t move_origin[STEPPER_COUNT];
static uint32_t wait_started, wait_ms, move_started, action_started;

uint8_t Actions_IsBusy(void)
{
    return status.state == ACTION_RUNNING || status.state == ACTION_CHASSIS_READY;
}
ActionStatus_t Actions_GetStatus(void) { return status; }

void Actions_Abort(uint8_t state)
{
    if (Actions_IsBusy() || status.state == ACTION_DONE) status.state = state;
    waiting_move = waiting_time = test_settle = move_parallel = 0;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    Stepper_Stop(H);
    Stepper_Stop(V);
    __set_PRIMASK(irq);
    Suction_AllOff();
}

uint8_t Actions_Start(uint32_t token, uint8_t id, uint8_t flags)
{
    if (token == 0 || id < ACTION_GRAP1 || id > ACTION_BUILD || flags > 1 ||
        (id == ACTION_BUILD && flags)) return ACK_ERR_PARAM;
    /* A replay can report the previous outcome but cannot restart a motion. */
    if (token == status.token)
        return id == status.id ? ACK_OK : ACK_ERR_PARAM;
    if (Actions_IsBusy() || Stepper_IsBusy(H) || Stepper_IsBusy(V))
        return ACK_ERR_BUSY;
    static const ActionStep *const sequences[] = {grap1, grap2, grap3, build};
    sequence = sequences[id - 1];
    status = (ActionStatus_t){token, id, ACTION_RUNNING, 0};
    waiting_move = waiting_time = move_parallel = 0;
    test_settle = flags;
    action_started = HAL_GetTick();
    return ACK_OK;
}

void Actions_Update(void)
{
    if (!Actions_IsBusy()) return;
    uint32_t now = HAL_GetTick();
    if (now - action_started >= 120000u) {
        Actions_Abort(ACTION_TIMEOUT);
        return;
    }
    if (waiting_move) {
        if (now - move_started >= 30000u) {
            Actions_Abort(ACTION_TIMEOUT);
            return;
        }
        if (Stepper_IsBusy(H) || Stepper_IsBusy(V)) {
            if (!move_parallel) return;
        } else {
            waiting_move = move_parallel = 0;
        }
        /* Continue immediately once both axes finish; no fixed settle delay. */
    }
    if (waiting_time) {
        if (now - wait_started < wait_ms) return;
        waiting_time = 0;
    }
    /* Consume instantaneous operations together, preserving servo overlap. */
    while (Actions_IsBusy()) {
        const ActionStep *s = &sequence[status.stage];
        const uint16_t *p = s->p;
        if (s->op == WAIT_PROGRESS) {
            /* Read pulse-count progress in the main loop, never block the ISR. */
            int64_t progress = (int64_t)Stepper_GetPosition(p[0]) - move_origin[p[0]];
            if (p[2] == R) progress = -progress;
            if (progress < p[1]) {
                if (!waiting_move) Actions_Abort(ACTION_TIMEOUT);
                return;
            }
        }
        if ((s->op == JOIN || s->op == END) && waiting_move) return;
        if (s->op == END) {
            if (test_settle) {
                test_settle = 0;
                waiting_time = 1;
                wait_started = now;
                wait_ms = 1000;
            } else status.state = ACTION_DONE;
            return;
        }
        status.stage++;
        switch (s->op) {
        case WAIT_PROGRESS: case JOIN: break;
        case CHASSIS_READY: status.state = ACTION_CHASSIS_READY; break;
        case HOME: Servo_HomeAll(); break;
        case HATCH:
            Servo_SetAngle(2, 130); Servo_SetAngle(3, 50); break;
        case ANGLE: Servo_SetAngleTenth(p[0], p[1]); break;
        case PUMP: Suction_PumpOn(); break;
        case RELEASE: Suction_Release(); break;
        case WAIT:
            waiting_time = 1; wait_started = now; wait_ms = p[0]; return;
        default: {
            /* A new move may not replace an unfinished parallel move. */
            if (waiting_move) { Actions_Abort(ACTION_CANCELLED); return; }
            /* TIM7 must not observe a partially initialized linked move. */
            uint32_t irq = __get_PRIMASK();
            __disable_irq();
            move_origin[H] = Stepper_GetPosition(H);
            move_origin[V] = Stepper_GetPosition(V);
            switch (s->op) {
            case MOVE:
                Stepper_StartMove(p[0], p[1], p[2], 1000, 83, 400); break;
            case DUAL: case DUAL_ASYNC:
                Stepper_StartMoveOverlap(p[0],p[1],p[2],p[3],p[4],p[5],p[6],
                                        1000,83,400); break;
            case DUAL2:
                Stepper_StartMoveOverlap2(p[0],p[1],p[2],p[3],p[4],p[5],p[6],
                                         p[7],p[8],1000,83,400); break;
            case DUAL3: case DUAL3_ASYNC:
                Stepper_StartMoveOverlap3(p[0],p[1],p[2],p[3],p[4],p[5],p[6],
                                         p[7],p[8],p[9],1000,83,400); break;
            }
            __set_PRIMASK(irq);
            waiting_move = 1; move_started = now;
            move_parallel = (s->op == DUAL_ASYNC || s->op == DUAL3_ASYNC);
            if (!move_parallel) return;
            break;
        }
        }
    }
}
