/**
 ******************************************************************************
 * @file    motor3508.c
 * @brief   M3508 motor control implementation
 ******************************************************************************
 */

#include "motor3508.h"
#include "can.h"
#include "gpio.h"
#include "imu.h"
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

    /* Motor feedback must be consumed by interrupt. At 50 Hz telemetry an
     * 80-byte UART frame blocks the main loop for about 7.5 ms, long enough
     * to overflow CAN FIFO0 and lose encoder deltas if reception is polled. */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
        Motor3508_RxPoll();
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
 * @brief  Speed PID directly producing the C620 current command
 * @note   The C620 already closes its motor-current loop. Adding another
 *         software current PID here halves low-speed torque and is not a
 *         valid cascade because torque_current is feedback, not a setpoint
 *         accepted by a separate actuator layer.
 */
void Motor3508_UpdateSpeedPID(uint8_t motor_id, float dt)
{
    if (motor_id >= M3508_COUNT || dt <= 0.0f) return;

    M3508_Motor_t *m = &g_motor[motor_id];

    int16_t torque_out = (int16_t)PID_Compute(&m->speed_pid,
                                               (float)m->target_speed,
                                               (float)m->feedback.speed_rpm,
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

/* ================== Telemetry / Feedback Access ============================ */

/**
 * @brief  Read motor feedback for telemetry packets
 */
void Motor3508_GetFeedback(uint8_t motor_id, uint16_t *angle,
                           int16_t *speed_rpm, int16_t *torque,
                           uint8_t *temperature)
{
    if (motor_id >= M3508_COUNT) return;
    M3508_Motor_t *m = &g_motor[motor_id];
    if (angle)       *angle       = m->feedback.angle;
    if (speed_rpm)   *speed_rpm   = m->feedback.speed_rpm;
    if (torque)      *torque      = m->feedback.torque_current;
    if (temperature) *temperature = m->feedback.temperature;
}

int32_t Motor3508_GetCumulativePosition(uint8_t motor_id)
{
    if (motor_id >= M3508_COUNT) return 0;
    return g_motor[motor_id].cumulative_pos;
}

/* ======================= Batch Speed Control =============================== */

/**
 * @brief  Set target speed for all 4 motors at once
 */
void Motor3508_SetAllSpeeds(const int16_t rpm[4])
{
    for (int i = 0; i < M3508_COUNT; i++)
        g_motor[i].target_speed = rpm[i];
}

/* ===================== Runtime PID Tuning ================================== */

/**
 * @brief  Adjust speed-loop PID gains at runtime
 */
void Motor3508_SetSpeedPID(uint8_t motor_id, float kp, float ki, float kd,
                           float integral_limit, float output_limit)
{
    if (motor_id >= M3508_COUNT) return;
    PID_t *p = &g_motor[motor_id].speed_pid;
    p->Kp             = kp;
    p->Ki             = ki;
    p->Kd             = kd;
    p->integral_limit = integral_limit;
    p->output_limit   = output_limit;
    p->integral       = 0.0f;   /* reset integrator on param change */
    p->prev_error     = 0.0f;
}

/**
 * @brief  Adjust position-loop PID gains at runtime
 */
void Motor3508_SetPosPID(uint8_t motor_id, float kp, float ki, float kd,
                         float integral_limit, float output_limit)
{
    if (motor_id >= M3508_COUNT) return;
    PID_t *p = &g_motor[motor_id].pos_pid;
    p->Kp             = kp;
    p->Ki             = ki;
    p->Kd             = kd;
    p->integral_limit = integral_limit;
    p->output_limit   = output_limit;
    p->integral       = 0.0f;
    p->prev_error     = 0.0f;
}

/**
 * @brief  Reset PID integrators for one motor
 */
void Motor3508_ResetPID(uint8_t motor_id)
{
    if (motor_id >= M3508_COUNT) return;
    g_motor[motor_id].speed_pid.integral   = 0.0f;
    g_motor[motor_id].speed_pid.prev_error = 0.0f;
    g_motor[motor_id].current_pid.integral = 0.0f;
    g_motor[motor_id].current_pid.prev_error = 0.0f;
    g_motor[motor_id].pos_pid.integral     = 0.0f;
    g_motor[motor_id].pos_pid.prev_error   = 0.0f;
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

/* =============== Position-Loop Distance Move ================================= */
/*
 *   S-curve feedforward + position-PID correction.  speed_rpm scales decel_dist
 *   and pid_limit linearly; accel time is fixed — cruise phase absorbs the gap.
 *
 *   settle + FWD_HOLD_MS position lock before exit.
 */

#define FWD_BASE_SPEED_RPM    1800.0f    /* reference speed for scaling */
#define FWD_ACCEL_MS           800       /* S-curve accel duration */
#define FWD_BASE_DECEL_DIST    90000     /* decel distance @ base speed (counts) */
#define FWD_BASE_PID_LIMIT     800.0f    /* PID authority @ base speed */
#define FWD_POS_KP               0.10f
#define FWD_POS_KI               0.003f
#define FWD_HOLD_MS            700       /* active lock after settle */
#define FWD_TIMEOUT_MS         5000      /* safety net */
#define FWD_YAW_KP             80.0f
#define FWD_YAW_KI              0.4f
#define FWD_YAW_MAX_RPM      1000.0f
#define FWD_SETTLE_COUNTS      400
#define FWD_SETTLE_CYCLES       50

/* =============== Turn (IMU-Based Rotation) =================================== */
/*
 *   S-curve feedforward + yaw-PID.  Uses IMU yaw as position reference;
 *   no encoder tracking.  IMU required — returns silently if not ready.
 *
 *   Conversion: °/s → motor RPM  (via mecanum rotation coupling)
 *     rot_rpm = ω(°/s) * half_diag(cm) * RPM_per_cm_s * π/180 ≈ ω * 13.14
 *   Mecanum formula: rpm[i] = ... - rot_rpm  (γ = [-1,-1,-1,-1])
 *     → CCW needs negative motor RPM, CW needs positive motor RPM
 */
#define TURN_DEG_S_TO_RPM      (MECANUM_RPM_PER_CM_S \
                                * (CHASSIS_HALF_DIAGONAL_MM / 10.0f) \
                                * 0.0174532925f)       /* π/180 */

#define TURN_BASE_SPEED_DEG_S   90.0f    /* reference angular speed */
#define TURN_ACCEL_MS           600      /* S-curve accel duration */
#define TURN_BASE_DECEL_DEG     30.0f    /* decel angle @ base speed */
#define TURN_BASE_PID_LIMIT     60.0f    /* PID correction limit @ base speed (°/s) */
#define TURN_POS_KP              3.0f    /* degree error → °/s correction */
#define TURN_POS_KI              0.15f
#define TURN_HOLD_MS            500      /* active lock after settle */
#define TURN_TIMEOUT_MS         5000     /* safety net */
#define TURN_SETTLE_DEG           1.5f   /* settle threshold (degrees) */
#define TURN_SETTLE_CYCLES        30

/* ---- smoothstep: r²(3-2r), zero-slope at both ends ------------------------ */
static inline float _smoothstep(float r) {
    return r * r * (3.0f - 2.0f * r);
}

/* ---- mm/s → motor RPM ---------------------------------------------------- */
static inline float _mm_s_to_rpm(float mm_s) {
    return mm_s * MECANUM_RPM_PER_CM_S / 10.0f;
}

/* ============================================================================ */

static void _move_linear(int32_t target, const int16_t sign[4], float speed_rpm)
{
    float dt    = 0.001f;
    float scale = speed_rpm / FWD_BASE_SPEED_RPM;
    float decel_dist = FWD_BASE_DECEL_DIST * scale;
    float pid_limit  = FWD_BASE_PID_LIMIT  * scale;

    Motor3508_ResetPosition();
    IMU_ResetYaw();

    for (int i = 0; i < M3508_COUNT; i++)
    {
        g_motor[i].speed_pid.integral   = 0.0f;
        g_motor[i].speed_pid.prev_error = 0.0f;
    }

    PID_t pos_pid;
    PID_Init(&pos_pid, FWD_POS_KP, FWD_POS_KI, 0.0f,
             pid_limit, pid_limit);

    PID_t yaw_pid;
    PID_Init(&yaw_pid, FWD_YAW_KP, FWD_YAW_KI, 0.0f,
             FWD_YAW_MAX_RPM, FWD_YAW_MAX_RPM);
    uint8_t imu_ok = IMU_IsReady();

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);

    uint32_t t_start    = HAL_GetTick();
    uint32_t settle_cnt = 0;
    uint8_t  hold_active = 0;
    uint32_t hold_start  = 0;

    while (1)
    {
        uint32_t t0 = HAL_GetTick();
        Motor3508_RxPoll();
        IMU_Update();

        /* ---- Combined reference: signed avg of all 4 motors ---- */
        int32_t ref_sum = 0;
        for (int i = 0; i < M3508_COUNT; i++)
            ref_sum += (int32_t)sign[i] * g_motor[i].cumulative_pos;
        int32_t ref = ref_sum / M3508_COUNT;
        if (ref < 0)  ref = 0;
        if (ref > target) ref = target;
        int32_t remaining = target - ref;

        /* ---- Feedforward (S-curve) + PID ---- */
        uint32_t elapsed = HAL_GetTick() - t_start;
        float    ff;

        if (elapsed < FWD_ACCEL_MS)
        {
            float r  = (float)elapsed / FWD_ACCEL_MS;
            pos_pid.output_limit = pid_limit * r;
            ff = speed_rpm * _smoothstep(r);
        }
        else
        {
            pos_pid.output_limit = pid_limit;
            if (remaining > decel_dist)
                ff = speed_rpm;
            else
                ff = speed_rpm * _smoothstep((float)remaining / decel_dist);
        }

        float pid_corr = PID_Compute(&pos_pid, (float)target, (float)ref, dt);
        float speed_sp = ff + pid_corr;

        /* ---- Yaw correction ---- */
        float yaw_corr = 0.0f;
        if (imu_ok)
            yaw_corr = PID_Compute(&yaw_pid, 0.0f, -IMU_GetYaw(), dt);

        /* ---- Distribute to 4 wheels + speed PID ---- */
        for (int i = 0; i < M3508_COUNT; i++)
        {
            float sp = speed_sp * sign[i] + yaw_corr;
            g_torque[i] = (int16_t)PID_Compute(&g_motor[i].speed_pid, sp,
                                                (float)g_motor[i].feedback.speed_rpm, dt);
        }
        Motor3508_SendTorques(g_torque);

        /* ---- Settle → hold (hold timer never resets) ---- */
        int32_t err = remaining;
        if (err < 0) err = -err;

        if (HAL_GetTick() - t_start >= FWD_TIMEOUT_MS)
            break;

        if (!hold_active)
        {
            if (err <= FWD_SETTLE_COUNTS)
            {
                if (++settle_cnt >= FWD_SETTLE_CYCLES)
                    hold_active = 1, hold_start = HAL_GetTick();
            }
            else
                settle_cnt = 0;
        }

        if (hold_active && (HAL_GetTick() - hold_start >= FWD_HOLD_MS))
            break;

        while (HAL_GetTick() - t0 < 1) { /* spin */ }
    }

    /* ---- Diagnostic: blink red LED = encoder error / 1000 counts ---- */
    int32_t final_sum = 0;
    for (int i = 0; i < M3508_COUNT; i++)
        final_sum += (int32_t)sign[i] * g_motor[i].cumulative_pos;
    int32_t final_err = target - (final_sum / M3508_COUNT);
    if (final_err < 0) final_err = -final_err;

    Motor3508_StopAll();

    uint32_t blinks = (uint32_t)(final_err / 1000);
    if (blinks > 20) blinks = 20;
    for (uint32_t b = 0; b < blinks; b++)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);  /* red ON */
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);    /* red OFF */
        HAL_Delay(100);
    }

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
}

/* ============================================================================ */

static void _turn(float target_deg, float speed_deg_s)
{
    /* User convention: + = CW (right), - = CCW (left).
     * Internally: + = CCW (matches IMU yaw).  Flip here so the rest
     * of the sign logic stays unchanged. */
    target_deg = -target_deg;

    if (!IMU_IsReady()) return;

    float dt           = 0.001f;
    int   sign_dir     = (target_deg >= 0.0f) ? 1 : -1;   /* +1=CCW, -1=CW */

    float scale        = speed_deg_s / TURN_BASE_SPEED_DEG_S;
    float decel_deg    = TURN_BASE_DECEL_DEG * scale;
    float pid_limit    = TURN_BASE_PID_LIMIT * scale;

    IMU_ResetYaw();

    for (int i = 0; i < M3508_COUNT; i++)
    {
        g_motor[i].speed_pid.integral   = 0.0f;
        g_motor[i].speed_pid.prev_error = 0.0f;
    }

    PID_t pos_pid;
    PID_Init(&pos_pid, TURN_POS_KP, TURN_POS_KI, 0.0f,
             pid_limit, pid_limit);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);  /* green ON */

    uint32_t t_start     = HAL_GetTick();
    uint32_t settle_cnt  = 0;
    uint8_t  hold_active = 0;
    uint32_t hold_start  = 0;

    while (1)
    {
        uint32_t t0 = HAL_GetTick();
        Motor3508_RxPoll();
        IMU_Update();

        float yaw       = IMU_GetYaw();          /* signed, since ResetYaw */
        float err       = target_deg - yaw;
        float remaining = (err >= 0.0f) ? err : -err;

        /* ---- Feedforward (S-curve) + PID ---- */
        uint32_t elapsed = HAL_GetTick() - t_start;
        float    ff_deg_s;

        if (elapsed < TURN_ACCEL_MS)
        {
            float r  = (float)elapsed / TURN_ACCEL_MS;
            pos_pid.output_limit = pid_limit * r;
            ff_deg_s = speed_deg_s * _smoothstep(r);
        }
        else
        {
            pos_pid.output_limit = pid_limit;
            if (remaining > decel_deg)
                ff_deg_s = speed_deg_s;
            else
                ff_deg_s = speed_deg_s * _smoothstep(remaining / decel_deg);
        }

        /* PID corrects signed angle error */
        float pid_deg_s  = PID_Compute(&pos_pid, target_deg, yaw, dt);
        float rot_deg_s  = ff_deg_s * sign_dir + pid_deg_s;

        /* Convert °/s → RPM, negate for mecanum γ=[-1,-1,-1,-1] convention */
        float rot_rpm = rot_deg_s * TURN_DEG_S_TO_RPM;
        for (int i = 0; i < M3508_COUNT; i++)
        {
            float sp = -rot_rpm;    /* all 4 motors same direction */
            g_torque[i] = (int16_t)PID_Compute(&g_motor[i].speed_pid, sp,
                                               (float)g_motor[i].feedback.speed_rpm, dt);
        }
        Motor3508_SendTorques(g_torque);

        /* ---- Settle → hold ---- */
        if (HAL_GetTick() - t_start >= TURN_TIMEOUT_MS)
            break;

        if (!hold_active)
        {
            if (remaining <= TURN_SETTLE_DEG)
            {
                if (++settle_cnt >= TURN_SETTLE_CYCLES)
                    hold_active = 1, hold_start = HAL_GetTick();
            }
            else
                settle_cnt = 0;
        }

        if (hold_active && (HAL_GetTick() - hold_start >= TURN_HOLD_MS))
            break;

        while (HAL_GetTick() - t0 < 1) { /* spin */ }
    }

    Motor3508_StopAll();
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);   /* green OFF */
}

/* =============== Public API ================================================== */

void Motor3508_MoveForward(float distance_mm, float speed_mm_s)
{
    int32_t target = (int32_t)(distance_mm * COUNTS_PER_CM / 10.0f);
    int16_t sign[4] = { -1, 1, 1, -1 };
    _move_linear(target, sign, _mm_s_to_rpm(speed_mm_s));
}

void Motor3508_MoveRight(float distance_mm, float speed_mm_s)
{
    int32_t target = (int32_t)(distance_mm * COUNTS_PER_CM / 10.0f);
    int16_t sign[4] = { 1, 1, -1, -1 };
    _move_linear(target, sign, _mm_s_to_rpm(speed_mm_s));
}

void Motor3508_Turn(float target_deg, float speed_deg_s)
{
    _turn(target_deg, speed_deg_s);
}
