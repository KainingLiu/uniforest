/**
 ******************************************************************************
 * @file    motor3508.h
 * @brief   M3508 motor control library for DJI A-board
 *          - CAN1 communication (TX on 0x200, RX on 0x201-0x208)
 *          - PID control framework (cascaded speed + current loop)
 *          - DC24V power management for C620 speed controllers
 ******************************************************************************
 */

#ifndef __MOTOR3508_H__
#define __MOTOR3508_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== CAN IDs =================================== */

#define CAN_M3508_CMD_ID           0x200u   /* TX: torque command for all 4 motors */
#define CAN_M3508_FEEDBACK_BASE    0x201u   /* RX: feedback from motor 1, up to 0x208 */

/* ========================= Motor Array Indices ============================ */
/*  Order matches the CAN data byte layout on 0x200 (big-endian pairs)        */
/*  ┌─────────┬──────────┐                                                  */
/*  │  Index  │ Position │                                                  */
/*  ├─────────┼──────────┤                                                  */
/*  │    0    │ 右上 (TR)│  CAN feedback ID: 0x201                           */
/*  │    1    │ 左上 (TL)│  CAN feedback ID: 0x202                           */
/*  │    2    │ 左下 (BL)│  CAN feedback ID: 0x203                           */
/*  │    3    │ 右下 (BR)│  CAN feedback ID: 0x204                           */
/*  └─────────┴──────────┘                                                  */
#define M3508_IDX_TR               0
#define M3508_IDX_TL               1
#define M3508_IDX_BL               2
#define M3508_IDX_BR               3
#define M3508_COUNT                4

/* =========================== Torque Limits ================================ */

#define M3508_TORQUE_MAX            16000
#define M3508_TORQUE_MIN           -16000

/* ======================== DC24V Power Control ============================= */

#define DC24V_PORT                  GPIOH
#define DC24V_PIN_1                 GPIO_PIN_2
#define DC24V_PIN_2                 GPIO_PIN_3
#define DC24V_PIN_3                 GPIO_PIN_4
#define DC24V_PIN_4                 GPIO_PIN_5
#define DC24V_PINS                  (DC24V_PIN_1 | DC24V_PIN_2 | \
                                     DC24V_PIN_3 | DC24V_PIN_4)
#define DC24V_STARTUP_DELAY_MS      500

/* ======================= Default PID Gains ================================ */
/*  Tunable — adjust after testing on actual chassis                          */
/*  Conservative defaults for light load                                       */

/* Position loop (outer): encoder counts → speed RPM */
#define M3508_POS_KP                0.02f
#define M3508_POS_KI                0.0005f /* accumulate to overcome friction */
#define M3508_POS_KD                0.0f
#define M3508_POS_INT_LIMIT         300.0f  /* max integral contribution (RPM) */
#define M3508_POS_OUT_LIMIT         600.0f

/* Speed loop (inner): RPM → torque (C620 handles current internally) */
#define M3508_SPEED_KP              10.0f
#define M3508_SPEED_KI              0.05f   /* help at low speed */
#define M3508_SPEED_KD              0.0f
#define M3508_SPEED_INT_LIMIT       6000.0f
#define M3508_SPEED_OUT_LIMIT       12000.0f

/* Current loop (reserved, not used in position cascade) */
#define M3508_CURRENT_KP            0.5f
#define M3508_CURRENT_KI            0.0f
#define M3508_CURRENT_KD            0.0f
#define M3508_CURRENT_INT_LIMIT     3000.0f
#define M3508_CURRENT_OUT_LIMIT     12000.0f

/* ========================== Data Structures =============================== */

/**
 * @brief Generic PID controller
 */
typedef struct {
    float  Kp;
    float  Ki;
    float  Kd;
    float  integral;
    float  integral_limit;      /* anti-windup clamp */
    float  output_limit;        /* output clamp */
    float  prev_error;
} PID_t;

/**
 * @brief M3508 motor feedback (decoded from CAN 0x201-0x208)
 *
 * Per the M3508 datasheet, each feedback frame is 8 bytes:
 *   [0:1]  rotor mechanical angle (0–8191)  — big-endian uint16
 *   [2:3]  rotational speed (RPM)            — big-endian int16
 *   [4:5]  actual torque current             — big-endian int16
 *   [6]    motor temperature (℃)             — uint8
 *   [7]    reserved (0x00)
 */
typedef struct {
    uint16_t angle;             /* raw encoder angle (0–8191 / rev) */
    int16_t  speed_rpm;         /* rotational speed in RPM */
    int16_t  torque_current;    /* actual torque current (raw) */
    uint8_t  temperature;       /* motor temperature in ℃ */
} M3508_Feedback_t;

/**
 * @brief Per-motor control context
 */
typedef struct {
    M3508_Feedback_t feedback;
    PID_t   pos_pid;            /* position loop: counts → speed RPM */
    PID_t   speed_pid;          /* speed loop: RPM → current setpoint */
    PID_t   current_pid;        /* current loop: current → torque output */
    int32_t target_position;    /* desired encoder position (counts) */
    int16_t target_speed;       /* desired speed (RPM, set by pos loop) */
    int32_t cumulative_pos;     /* accumulated encoder counts */
    int16_t last_angle;         /* previous raw angle for delta calc */
    uint8_t angle_valid;        /* flag: first reading received */
} M3508_Motor_t;

/* ====================== Chassis Geometry ================================= */
/*  Wheelbase (前后轴距): 240 mm, Track width (左右轮距): 391 mm               */
/*  Half-diagonal for rotation coupling: (W+L)/2 = (391+240)/2 = 315.5 mm      */
/*  Wheel: D=152mm, gear 19:1, encoder 8192/rev                                */
/*  Right wheels (idx 0,3): positive RPM = clockwise (CW)                       */
/*  Left wheels  (idx 1,2): positive RPM = counter-clockwise (CCW)              */

#define ENCODER_COUNTS_PER_REV      8192
#define M3508_GEAR_RATIO              19
#define WHEEL_DIAMETER_MM            152.0f
#define WHEEL_CIRCUMFERENCE_MM       (3.1415926535f * WHEEL_DIAMETER_MM)
#define WHEEL_RADIUS_MM              (WHEEL_DIAMETER_MM / 2.0f)

#define CHASSIS_WHEELBASE_MM         240.0f
#define CHASSIS_TRACK_MM             391.0f
#define CHASSIS_HALF_DIAGONAL_MM     ((CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) / 2.0f)

/* Conversion: v(cm/s) → motor RPM  */
/* RPM = v_cm_s * 60 * 19 / (π * 15.2) = v_cm_s * 23.873...                   */
#define MECANUM_RPM_PER_CM_S         (60.0f * M3508_GEAR_RATIO / (3.1415926535f * (WHEEL_DIAMETER_MM / 10.0f)))

/* Rotation contribution: ω(rad/s) * half_diagonal(cm) * RPM_PER_CM_S          */
/* half_diag = 31.55 cm; conversion ≈ 753 RPM per rad/s                        */

/* counts per cm of chassis travel */
#define COUNTS_PER_CM   ((int32_t)(10.0f * M3508_GEAR_RATIO \
                                   * ENCODER_COUNTS_PER_REV \
                                   / WHEEL_CIRCUMFERENCE_MM))

/* =================== Legacy Demo Parameters (unused, kept for reference) === */

#define DEMO_DISTANCE_CM              20
#define DEMO_TARGET_COUNTS           ((int32_t)(DEMO_DISTANCE_CM * COUNTS_PER_CM))
#define DEMO_PAUSE_MS                1000
#define DEMO_SETTLE_THRESHOLD        2000
#define DEMO_ACCEL_MS                300
#define DEMO_CRUISE_MS               800
#define DEMO_DECEL_MS                300
#define DEMO_SPEED_RPM               800
#define DEMO_HOLD_MS                 200

#define PID_DT                       0.001f

/* ========================== Public API ==================================== */

/* ---------- Initialization ---------- */
void Motor3508_Init(void);

/* ---------- CAN Transmission ---------- */
void Motor3508_SendTorques(const int16_t torque[4]);

/* ---------- CAN Reception (poll mode) ---------- */
void Motor3508_RxPoll(void);

/* ---------- Position Control ---------- */
void Motor3508_SetPosition(uint8_t motor_id, int32_t target_counts);

/* ---------- Open-Loop Control ---------- */
void Motor3508_SetCurrent(uint8_t motor_id, int16_t current);
void Motor3508_SetSpeed(uint8_t motor_id, int16_t speed_rpm);

/* ---------- Cascaded PID (pos→speed→current) ---------- */
void Motor3508_UpdatePositionPID(uint8_t motor_id, float dt);
void Motor3508_UpdateAllPositionPID(float dt);
void Motor3508_UpdateTrapezoidPID(uint8_t motor_id, float dt);
void Motor3508_UpdateAllTrapezoidPID(float dt);
void Motor3508_UpdateSpeedPID(uint8_t motor_id, float dt);
void Motor3508_UpdateAllSpeedPID(float dt);

/* ---------- Emergency / Shutdown ---------- */
void Motor3508_StopAll(void);

/* ---------- Power Management ---------- */
void Motor3508_PowerOn(void);
void Motor3508_PowerOff(void);

/* ---------- PID Utilities ---------- */
void   PID_Init(PID_t *pid, float kp, float ki, float kd,
                float integral_limit, float output_limit);
float  PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);

/* ---------- Chassis Demo ---------- */
void Motor3508_ChassisDemo(void);          /* legacy 20cm trapezoid demo */
void Place(void);                          /* 4-click: forward→hatch→back→close */

/* ---------- Chassis Demo ---------- */
void Motor3508_ChassisDemo1M(void);         /* 8-direction test */

/* ---------- Mecanum Kinematics ---------- */
/**
 * @brief  Convert chassis velocity to 4 wheel RPMs
 * @param  vx_cm_s    forward velocity in cm/s (+=forward)
 * @param  vy_cm_s    lateral velocity in cm/s (+=right, empirically verified)
 * @param  wz_deg_s   angular velocity in °/s (+=CCW from top view)
 * @param  rpm_out[4] output wheel RPMs (per motor sign convention)
 * @note   Rotation sign pattern γ = [-1, -1, -1, -1] for TR,TL,BL,BR.
 *         For user convention wz += CW, negate wz_deg_s before calling.
 */
void Motor3508_MecanumRPM(float vx_cm_s, float vy_cm_s, float wz_deg_s,
                          float rpm_out[4]);

/* ---------- Position Tracking ---------- */
int32_t Motor3508_GetAvgPosition(void);
void    Motor3508_ResetPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR3508_H__ */
