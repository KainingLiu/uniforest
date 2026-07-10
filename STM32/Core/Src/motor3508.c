/**
 ******************************************************************************
 * @file    motor3508.c
 * @brief   M3508 motor control implementation
 ******************************************************************************
 */

#include "motor3508.h"
#include "can.h"
#include "gpio.h"
#include "actions.h"
#include <string.h>

/* ============================ Private Variables =========================== */

static int16_t  g_torque[4] = {0};           /* current torque command for each motor */
static M3508_Motor_t g_motor[4];             /* per-motor control context */

/* ========================== PID Implementation ============================ */

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->Kp             = kp;
    pid->Ki             = ki;
    pid->Kd             = kd;
    pid->integral       = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->prev_error     = 0.0f;
}

float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* Proportional term */
    float p_term = pid->Kp * error;

    /* Integral term (with anti-windup clamping) */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    float i_term = pid->Ki * pid->integral;

    /* Derivative term (with dt guard against division by zero) */
    float d_term = 0.0f;
    if (dt > 0.000001f)
    {
        float derivative = (error - pid->prev_error) / dt;
        d_term = pid->Kd * derivative;
    }
    pid->prev_error = error;

    /* Sum and clamp output */
    float output = p_term + i_term + d_term;
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    return output;
}

/* ====================== CAN Transmission ================================== */

/**
 * @brief  Pack four int16 torque values and send on CAN1 StdId 0x200
 * @note   Big-endian byte order per M3508/C620 protocol:
 *         [0:1]=motor1, [2:3]=motor2, [4:5]=motor3, [6:7]=motor4
 *         Each pair: high byte first, then low byte.
 */
void Motor3508_SendTorques(const int16_t torque[4])
{
    /* Clamp and pack (big-endian) */
    int16_t t[4];
    for (int i = 0; i < 4; i++)
    {
        if (torque[i] > M3508_TORQUE_MAX)  t[i] = M3508_TORQUE_MAX;
        else if (torque[i] < M3508_TORQUE_MIN) t[i] = M3508_TORQUE_MIN;
        else t[i] = torque[i];
    }

    uint8_t data[8] = {
        (uint8_t)((t[0] >> 8) & 0xFF), (uint8_t)(t[0] & 0xFF),
        (uint8_t)((t[1] >> 8) & 0xFF), (uint8_t)(t[1] & 0xFF),
        (uint8_t)((t[2] >> 8) & 0xFF), (uint8_t)(t[2] & 0xFF),
        (uint8_t)((t[3] >> 8) & 0xFF), (uint8_t)(t[3] & 0xFF),
    };

    CAN_TxHeaderTypeDef tx = {
        .StdId = CAN_M3508_CMD_ID,
        .IDE   = CAN_ID_STD,
        .RTR   = CAN_RTR_DATA,
        .DLC   = 8,
    };
    uint32_t mb;
    HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mb);
}

/* ====================== CAN Reception (Poll Mode) ========================= */

/**
 * @brief  Poll CAN1 RX FIFO for motor feedback messages (IDs 0x201–0x208)
 * @note   Call this periodically from main loop or a timer ISR.
 *         Decodes each 8-byte feedback frame:
 *           [0:1] angle (uint16, big-endian, 0–8191)
 *           [2:3] speed  (int16, big-endian, RPM)
 *           [4:5] torque (int16, big-endian, raw current)
 *           [6]   temperature (uint8, Celsius)
 *           [7]   reserved
 */
void Motor3508_RxPoll(void)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, data) != HAL_OK)
            break;

        /* Feedback IDs: 0x201 = motor 1, ..., 0x208 = motor 8 */
        if (rx.StdId < CAN_M3508_FEEDBACK_BASE ||
            rx.StdId > CAN_M3508_FEEDBACK_BASE + 7)
            continue;   /* not a motor feedback frame */

        uint8_t idx = (uint8_t)(rx.StdId - CAN_M3508_FEEDBACK_BASE);
        if (idx >= M3508_COUNT)
            continue;   /* only care about motors 0–3 */

        uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];

        g_motor[idx].feedback.angle         = raw_angle;
        g_motor[idx].feedback.speed_rpm     =  (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        g_motor[idx].feedback.torque_current=  (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        g_motor[idx].feedback.temperature   = data[6];

        /* ---- Cumulative position tracking (handles 0–8191 wrap) ---- */
        if (g_motor[idx].angle_valid)
        {
            int16_t delta = (int16_t)(raw_angle - g_motor[idx].last_angle);

            /* Detect wrap: if |delta| > 4096, it's a wrap, not real movement */
            if (delta > 4096)
                delta -= ENCODER_COUNTS_PER_REV;      /* wrapped down → negative */
            else if (delta < -4096)
                delta += ENCODER_COUNTS_PER_REV;      /* wrapped up → positive */

            g_motor[idx].cumulative_pos += delta;
        }
        else
        {
            g_motor[idx].angle_valid = 1;   /* first reading — initialize */
        }
        g_motor[idx].last_angle = raw_angle;
    }
}

/* ======================== Initialization ================================== */

void Motor3508_Init(void)
{
    /* 1. Start CAN1 peripheral (already initialized by MX_CAN1_Init) */
    HAL_CAN_Start(&hcan1);

    /* 2. Configure CAN filter to accept all standard IDs (bank 0)
     *    We filter the specific motor IDs in software inside RxPoll() */
    CAN_FilterTypeDef filter = {
        .FilterIdHigh        = 0x0000,
        .FilterIdLow         = 0x0000,
        .FilterMaskIdHigh    = 0x0000,
        .FilterMaskIdLow     = 0x0000,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterBank          = 0,
        .FilterMode          = CAN_FILTERMODE_IDMASK,
        .FilterScale         = CAN_FILTERSCALE_32BIT,
        .FilterActivation    = ENABLE,
        .SlaveStartFilterBank= 14,
    };
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    /* 3. Enable DC24V power for C620 motor controllers */
    Motor3508_PowerOn();
    HAL_Delay(DC24V_STARTUP_DELAY_MS);

    /* 4. Initialize PID structures for all 4 motors */
    for (int i = 0; i < M3508_COUNT; i++)
    {
        memset(&g_motor[i], 0, sizeof(M3508_Motor_t));

        /* Position loop (outermost): counts → speed RPM */
        PID_Init(&g_motor[i].pos_pid,
                 M3508_POS_KP, M3508_POS_KI, M3508_POS_KD,
                 M3508_POS_INT_LIMIT, M3508_POS_OUT_LIMIT);

        /* Speed loop (middle): RPM → current */
        PID_Init(&g_motor[i].speed_pid,
                 M3508_SPEED_KP, M3508_SPEED_KI, M3508_SPEED_KD,
                 M3508_SPEED_INT_LIMIT, M3508_SPEED_OUT_LIMIT);

        /* Current loop (inner): current → torque */
        PID_Init(&g_motor[i].current_pid,
                 M3508_CURRENT_KP, M3508_CURRENT_KI, M3508_CURRENT_KD,
                 M3508_CURRENT_INT_LIMIT, M3508_CURRENT_OUT_LIMIT);
    }

    /* 5. Send zero torque to all motors */
    Motor3508_StopAll();
}

/* ====================== Control Interfaces ================================ */

/**
 * @brief  Set target torque current for a single motor (open-loop mode)
 * @param  motor_id  M3508_IDX_TR (0) … M3508_IDX_BR (3)
 * @param  current   torque current in raw units (-16000 … 16000)
 */
void Motor3508_SetCurrent(uint8_t motor_id, int16_t current)
{
    if (motor_id >= M3508_COUNT) return;

    /* Clamp */
    if (current > M3508_TORQUE_MAX) current = M3508_TORQUE_MAX;
    if (current < M3508_TORQUE_MIN) current = M3508_TORQUE_MIN;

    g_torque[motor_id] = current;
    Motor3508_SendTorques(g_torque);
}

/**
 * @brief  Set target speed for PID closed-loop control
 * @param  motor_id   M3508_IDX_TR (0) … M3508_IDX_BR (3)
 * @param  speed_rpm  desired speed in RPM
 */
void Motor3508_SetSpeed(uint8_t motor_id, int16_t speed_rpm)
{
    if (motor_id >= M3508_COUNT) return;
    g_motor[motor_id].target_speed = speed_rpm;
}

/* ====================== Position Control ================================== */

/**
 * @brief  Set position target for a motor (encoder counts)
 * @param  motor_id        M3508_IDX_TR (0) … M3508_IDX_BR (3)
 * @param  target_counts   desired cumulative encoder position
 */
void Motor3508_SetPosition(uint8_t motor_id, int32_t target_counts)
{
    if (motor_id >= M3508_COUNT) return;
    g_motor[motor_id].target_position = target_counts;
}

/**
 * @brief  Cascaded position→speed PID (C620 handles current internally)
 * @param  motor_id  M3508_IDX_TR (0) … M3508_IDX_BR (3)
 * @param  dt        time delta since last call (seconds)
 */
void Motor3508_UpdatePositionPID(uint8_t motor_id, float dt)
{
    if (motor_id >= M3508_COUNT || dt <= 0.0f) return;

    M3508_Motor_t *m = &g_motor[motor_id];

    /* Layer 1 — Position loop: encoder counts → speed setpoint (RPM) */
    float speed_sp = PID_Compute(&m->pos_pid,
                                  (float)m->target_position,
                                  (float)m->cumulative_pos,
                                  dt);

    /* Layer 2 — Speed loop: RPM → torque output (direct to CAN) */
    int16_t torque_out = (int16_t)PID_Compute(&m->speed_pid,
                                               speed_sp,
                                               (float)m->feedback.speed_rpm,
                                               dt);

    g_torque[motor_id] = torque_out;
    Motor3508_SendTorques(g_torque);
}

void Motor3508_UpdateAllPositionPID(float dt)
{
    for (uint8_t i = 0; i < M3508_COUNT; i++)
        Motor3508_UpdatePositionPID(i, dt);
}

/* ====================== Speed-Only PID ==================================== */

/**
 * @brief  Speed→current cascaded PID (no position loop)
 */
void Motor3508_UpdateSpeedPID(uint8_t motor_id, float dt)
{
    if (motor_id >= M3508_COUNT || dt <= 0.0f) return;

    M3508_Motor_t *m = &g_motor[motor_id];

    float current_sp = PID_Compute(&m->speed_pid,
                                    (float)m->target_speed,
                                    (float)m->feedback.speed_rpm,
                                    dt);

    int16_t torque_out = (int16_t)PID_Compute(&m->current_pid,
                                               current_sp,
                                               (float)m->feedback.torque_current,
                                               dt);

    g_torque[motor_id] = torque_out;
    Motor3508_SendTorques(g_torque);
}

void Motor3508_UpdateAllSpeedPID(float dt)
{
    for (uint8_t i = 0; i < M3508_COUNT; i++)
        Motor3508_UpdateSpeedPID(i, dt);
}

/* ========================== Emergency Stop ================================ */

void Motor3508_StopAll(void)
{
    int16_t zero[4] = {0, 0, 0, 0};
    Motor3508_SendTorques(zero);
    g_torque[0] = g_torque[1] = g_torque[2] = g_torque[3] = 0;

    /* Reset PID integrators */
    for (int i = 0; i < M3508_COUNT; i++)
    {
        g_motor[i].speed_pid.integral   = 0.0f;
        g_motor[i].speed_pid.prev_error = 0.0f;
        g_motor[i].current_pid.integral = 0.0f;
        g_motor[i].current_pid.prev_error = 0.0f;
    }
}

/* ======================== Power Management ================================ */

void Motor3508_PowerOn(void)
{
    HAL_GPIO_WritePin(DC24V_PORT, DC24V_PINS, GPIO_PIN_SET);
}

void Motor3508_PowerOff(void)
{
    HAL_GPIO_WritePin(DC24V_PORT, DC24V_PINS, GPIO_PIN_RESET);
}

/* ======================= Position Tracking ================================ */

/**
 * @brief  Get average cumulative encoder position across all 4 motors
 * @return average encoder counts (forward = positive)
 */
int32_t Motor3508_GetAvgPosition(void)
{
    int32_t sum = 0;
    for (int i = 0; i < M3508_COUNT; i++)
    {
        sum += g_motor[i].cumulative_pos;
    }
    return sum / M3508_COUNT;
}

/**
 * @brief  Reset cumulative position tracking for all motors
 */
void Motor3508_ResetPosition(void)
{
    for (int i = 0; i < M3508_COUNT; i++)
    {
        g_motor[i].cumulative_pos = 0;
        g_motor[i].last_angle     = (int16_t)g_motor[i].feedback.angle;
        g_motor[i].angle_valid    = 0;  /* will re-init on next RxPoll */
    }
}

/* ======================== Chassis Demo ==================================== */

/**
 * @brief  Time-based trapezoidal velocity profile (extended: configurable cruise)
 * @param  dir[4]    target RPM array (signed, includes direction)
 * @param  cruise_ms cruise duration in ms
 */
static void _trapezoid_run_ex(int16_t dir[4], uint32_t cruise_ms)
{
    uint32_t t, t0;

    /* Accel */
    t0 = HAL_GetTick();
    while ((t = HAL_GetTick() - t0) < DEMO_ACCEL_MS) {
        float r = (float)t / DEMO_ACCEL_MS;
        for (int i = 0; i < M3508_COUNT; i++)
            Motor3508_SetSpeed(i, (int16_t)(dir[i] * r));
        Motor3508_RxPoll(); Motor3508_UpdateAllSpeedPID(PID_DT);
        HAL_Delay(3);
    }
    /* Cruise */
    t0 = HAL_GetTick();
    while ((t = HAL_GetTick() - t0) < cruise_ms) {
        for (int i = 0; i < M3508_COUNT; i++)
            Motor3508_SetSpeed(i, dir[i]);
        Motor3508_RxPoll(); Motor3508_UpdateAllSpeedPID(PID_DT);
        HAL_Delay(3);
    }
    /* Decel */
    t0 = HAL_GetTick();
    while ((t = HAL_GetTick() - t0) < DEMO_DECEL_MS) {
        float r = 1.0f - (float)t / DEMO_DECEL_MS;
        for (int i = 0; i < M3508_COUNT; i++)
            Motor3508_SetSpeed(i, (int16_t)(dir[i] * r));
        Motor3508_RxPoll(); Motor3508_UpdateAllSpeedPID(PID_DT);
        HAL_Delay(3);
    }
    /* Hold zero */
    for (int i = 0; i < M3508_COUNT; i++) {
        Motor3508_SetSpeed(i, 0);
        g_motor[i].speed_pid.integral = 0.0f;
    }
    t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < DEMO_HOLD_MS) {
        Motor3508_RxPoll(); Motor3508_UpdateAllSpeedPID(PID_DT);
        HAL_Delay(3);
    }
    Motor3508_StopAll();
}

/* Legacy 20cm demo (kept for reference) */
void Motor3508_ChassisDemo(void)
{
    int16_t fwd[4] = { -DEMO_SPEED_RPM,  DEMO_SPEED_RPM,
                        DEMO_SPEED_RPM, -DEMO_SPEED_RPM };
    int16_t rev[4] = {  DEMO_SPEED_RPM, -DEMO_SPEED_RPM,
                       -DEMO_SPEED_RPM,  DEMO_SPEED_RPM };

    _trapezoid_run_ex(fwd, DEMO_CRUISE_MS);
    Actions_HatchOpen();
    HAL_Delay(DEMO_PAUSE_MS);
    _trapezoid_run_ex(rev, DEMO_CRUISE_MS);
    Actions_HatchClose();
}

/* ============ Place() — original 20cm pick-and-place demo ================== */

void Place(void)
{
    int16_t fwd[4] = { -DEMO_SPEED_RPM,  DEMO_SPEED_RPM,
                        DEMO_SPEED_RPM, -DEMO_SPEED_RPM };
    int16_t rev[4] = {  DEMO_SPEED_RPM, -DEMO_SPEED_RPM,
                       -DEMO_SPEED_RPM,  DEMO_SPEED_RPM };

    _trapezoid_run_ex(fwd, DEMO_CRUISE_MS + 150);  /* +150ms ≈ +5cm */
    Actions_HatchOpen();
    HAL_Delay(DEMO_PAUSE_MS);
    _trapezoid_run_ex(rev, DEMO_CRUISE_MS + 150);
    Actions_HatchClose();
}

/* ======== 1-Click Demo: 8-direction comprehensive test ==================== */
/*
 *  1. Forward 1m        {- + + -},  cruise 2700
 *  2. Backward 1m       {+ - - +},  cruise 2700
 *  3. RF 45° 1m         {0 + 0 -},  cruise 3700
 *  4. LF 45° 1m         {- 0 + 0},  cruise 3700
 *  5. LB 45° 1m         {0 - 0 +},  cruise 3700
 *  6. RB 45° 1m         {+ 0 - 0},  cruise 3700
 *  7. Right lateral 1m  {+ + - -},  cruise 2700  (4-wheel pure lateral)
 *  8. Left lateral 1m   {- - + +},  cruise 2700
 */

#define STRAIGHT_CRUISE  2700
#define DIAG_CRUISE_MS   3700

void Motor3508_ChassisDemo1M(void)
{
    /* ---------- straight ---------- */
    int16_t fwd[4] = { -DEMO_SPEED_RPM,  DEMO_SPEED_RPM,
                        DEMO_SPEED_RPM, -DEMO_SPEED_RPM };
    int16_t rev[4] = {  DEMO_SPEED_RPM, -DEMO_SPEED_RPM,
                       -DEMO_SPEED_RPM,  DEMO_SPEED_RPM };

    /* ---------- diagonal ---------- */
    int16_t rf[4] = { 0,  DEMO_SPEED_RPM, 0, -DEMO_SPEED_RPM };  /* TL+ BR- */
    int16_t lf[4] = { -DEMO_SPEED_RPM, 0,  DEMO_SPEED_RPM, 0 };  /* TR- BL+ */
    int16_t lb[4] = { 0, -DEMO_SPEED_RPM, 0,  DEMO_SPEED_RPM };  /* TL- BR+ */
    int16_t rb[4] = {  DEMO_SPEED_RPM, 0, -DEMO_SPEED_RPM, 0 };  /* TR+ BL- */

    /* ---------- lateral ---------- */
    int16_t lat_r[4] = {  DEMO_SPEED_RPM,  DEMO_SPEED_RPM,
                         -DEMO_SPEED_RPM, -DEMO_SPEED_RPM };  /* all: right */
    int16_t lat_l[4] = { -DEMO_SPEED_RPM, -DEMO_SPEED_RPM,
                          DEMO_SPEED_RPM,  DEMO_SPEED_RPM };  /* all: left */

    /* 1 */ Motor3508_ResetPosition(); _trapezoid_run_ex(fwd,  STRAIGHT_CRUISE); HAL_Delay(300);
    /* 2 */ Motor3508_ResetPosition(); _trapezoid_run_ex(rev,  STRAIGHT_CRUISE); HAL_Delay(300);
    /* 3 */ Motor3508_ResetPosition(); _trapezoid_run_ex(rf,   DIAG_CRUISE_MS);  HAL_Delay(300);
    /* 4 */ Motor3508_ResetPosition(); _trapezoid_run_ex(lf,   DIAG_CRUISE_MS);  HAL_Delay(300);
    /* 5 */ Motor3508_ResetPosition(); _trapezoid_run_ex(lb,   DIAG_CRUISE_MS);  HAL_Delay(300);
    /* 6 */ Motor3508_ResetPosition(); _trapezoid_run_ex(rb,   DIAG_CRUISE_MS);  HAL_Delay(300);
    /* 7 */ Motor3508_ResetPosition(); _trapezoid_run_ex(lat_r, STRAIGHT_CRUISE); HAL_Delay(300);
    /* 8 */ Motor3508_ResetPosition(); _trapezoid_run_ex(lat_l, STRAIGHT_CRUISE);
}

/* ======================== Mecanum Kinematics =============================== */

/**
 * @brief  Convert chassis velocity to 4 wheel RPMs
 *
 * Mecanum inverse kinematics with sign convention:
 *   Right wheels (idx 0,3): positive RPM = CW
 *   Left wheels  (idx 1,2): positive RPM = CCW
 *
 * Wheel layout (top view, robot facing +x):
 *          ┌──────┬──────┐
 *          │ TL(1)│ TR(0)│  ← front
 *          │  +fwd │  -fwd │
 *          ├──────┼──────┤
 *          │ BL(2)│ BR(3)│  ← rear
 *          │  +fwd │  -fwd │
 *          └──────┴──────┘
 *
 * Geometry: wheelbase=240mm, track=391mm, wheel D=152mm, gear 19:1
 *
 * The standard mecanum formula (X-roller config, all CCW=forward):
 *   v_std[0] = +vx - vy - L*wz    (TR)
 *   v_std[1] = +vx + vy + L*wz    (TL)
 *   v_std[2] = +vx + vy - L*wz    (BL)
 *   v_std[3] = +vx - vy + L*wz    (BR)
 *   where L = half_wheelbase + half_track
 *
 * Motor sign adapt: TR→negate, TL→keep, BL→keep, BR→negate
 * Result in motor RPM convention:
 *   rpm[0] = -vx + vy + L*wz     (TR: -forward, +left, +CCW-rot)
 *   rpm[1] = +vx + vy + L*wz     (TL: +forward, +left, +CCW-rot)
 *   rpm[2] = +vx - vy + L*wz     (BL: +forward, -left, +CCW-rot)
 *   rpm[3] = -vx - vy + L*wz     (BR: -forward, -left, +CCW-rot)
 *
 * @param  vx_cm_s    forward velocity in cm/s (+=forward)
 * @param  vy_cm_s    lateral velocity in cm/s (+=left)
 * @param  wz_deg_s   angular velocity in °/s (+=CCW from top view)
 * @param  rpm_out[4] output wheel RPMs (motor sign convention)
 */
void Motor3508_MecanumRPM(float vx_cm_s, float vy_cm_s, float wz_deg_s,
                          float rpm_out[4])
{
    /* Convert chassis velocity (cm/s) → nominal wheel RPM */
    float rpm_per_cm_s = MECANUM_RPM_PER_CM_S;  /* ≈ 23.87 */

    float base_rpm  = vx_cm_s * rpm_per_cm_s;
    float lat_rpm   = vy_cm_s * rpm_per_cm_s;

    /* Convert angular velocity (°/s) → rad/s */
    float wz_rad_s  = wz_deg_s * 0.0174532925f;   /* π/180 */

    /* Rotation RPM contribution:
     *   RPM_rot = ω(rad/s) * half_diag(cm) * RPM_per_cm_s
     *   half_diag = (wheelbase + track) / 2 = 31.55 cm
     *   → RPM_rot ≈ ω(rad/s) * 753.5
     */
    float rot_rpm   = wz_rad_s * (CHASSIS_HALF_DIAGONAL_MM / 10.0f) * rpm_per_cm_s;

    /* Mecanum inverse kinematics (motor sign convention)
     * Rotation sign pattern γ = [-1, -1, -1, -1] — verified by
     * pseudo-inverse: (MᵀM) is diagonal → vx/vy/ωz are decoupled.
     */
    rpm_out[0] = -base_rpm + lat_rpm - rot_rpm;  /* TR: γ₀=-1 */
    rpm_out[1] = +base_rpm + lat_rpm - rot_rpm;  /* TL: γ₁=-1 */
    rpm_out[2] = +base_rpm - lat_rpm - rot_rpm;  /* BL: γ₂=-1 */
    rpm_out[3] = -base_rpm - lat_rpm - rot_rpm;  /* BR: γ₃=-1 */
}
