/**
 ******************************************************************************
 * @file    servo.h
 * @brief   Servo PWM control library for DJI A-board
 *          4 servos via TIM2 (CH2/CH3/CH4) + TIM5 (CH3)
 *          50 Hz PWM, 0.5–2.5 ms pulse → 0–180°
 *
 *  Pin mapping (from user's T–Z table):
 *  ┌──────────┬────┬──────┬───────────────┬──────┐
 *  │  Servo   │ ID │ Pin  │ Timer/Channel │  AF  │
 *  ├──────────┼────┼──────┼───────────────┼──────┤
 *  │ S1 夹爪  │  0 │ PA1  │ TIM2_CH2      │ AF1  │
 *  │ S2 前臂  │  1 │ PA2  │ TIM2_CH3      │ AF1  │
 *  │ S3 舱门A │  2 │ PA3  │ TIM2_CH4      │ AF1  │
 *  │ S4 舱门B │  3 │ PA0  │ TIM5_CH1      │ AF2  │
 *  └──────────┴────┴──────┴───────────────┴──────┘
 ******************************************************************************
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Timer Parameters ================================ */
/*
 * Clock: APB1 timer clock = 90 MHz  (APB1 = 45 MHz, ×2 when prescaler ≠ 1)
 * Target: 50 Hz PWM  (20 ms period)
 *
 * 90 MHz / (PSC+1) = 500 kHz   →  PSC = 179
 * 500 kHz / (ARR+1) = 50 Hz    →  ARR = 9999
 *
 * Pulse range at 500 kHz (1 tick = 2 µs):
 *   0.5 ms →  250 counts   (0°)
 *   1.5 ms →  750 counts   (90°)
 *   2.5 ms → 1250 counts   (180°)
 */

#define SERVO_TIM_PSC               179
#define SERVO_TIM_ARR               9999

#define SERVO_PULSE_MIN             250     /* 0.5 ms → 0° */
#define SERVO_PULSE_MID             750     /* 1.5 ms → 90° */
#define SERVO_PULSE_MAX             1250    /* 2.5 ms → 180° */

/* ========================== Servo IDs ===================================== */

#define SERVO_GRIPPER               0       /* S1: 夹爪 */
#define SERVO_ARM_FRONT             1       /* S2: 前臂 */
#define SERVO_HATCH_A               2       /* S3: 舱门A */
#define SERVO_HATCH_B               3       /* S4: 舱门B */
#define SERVO_COUNT                 4

/* ======================== Home Angles ===================================== */

#define SERVO1_HOME                 81      /* gripper open */
#define SERVO2_HOME                 90      /* arm front down */
#define SERVO3_HOME                 63      /* hatch A closed */
#define SERVO4_HOME                 117     /* hatch B closed */

/* ======================== Angle Limits ==================================== */

#define SERVO_ANGLE_MIN             0
#define SERVO_ANGLE_MAX             180

/* ==================== Timer / Channel Assignments ========================= */
/*  Change these defines to re-map a servo to a different timer channel      */

#define SERVO0_TIM                  TIM2
#define SERVO0_CHANNEL              TIM_CHANNEL_2

#define SERVO1_TIM                  TIM2
#define SERVO1_CHANNEL              TIM_CHANNEL_3

#define SERVO2_TIM                  TIM2
#define SERVO2_CHANNEL              TIM_CHANNEL_4

#define SERVO3_TIM                  TIM5
#define SERVO3_CHANNEL              TIM_CHANNEL_1

/* ========================== Public API ==================================== */

/**
 * @brief  Initialize all 4 servo PWM outputs
 * @note   Configures PA1/PA2/PA3 (AF1 → TIM2) and PI2 (AF2 → TIM5)
 *         as alternate-function push-pull outputs.
 *         Starts PWM on all channels with SERVO_PULSE_MID as initial value.
 */
void Servo_Init(void);

/**
 * @brief  Set a servo to a specific angle
 * @param  servo_id   SERVO_GRIPPER (0) … SERVO_HATCH_B (3)
 * @param  angle_deg  0–180 degrees
 * @note   Linear mapping: pulse = MIN + (angle * (MAX-MIN)) / 180
 *         Clamped to [SERVO_ANGLE_MIN, SERVO_ANGLE_MAX].
 */
void Servo_SetAngle(uint8_t servo_id, uint8_t angle_deg);

/**
 * @brief  Set all servos to their home (idle) positions
 *         S1=81, S2=90, S3=63, S4=117
 */
void Servo_HomeAll(void);

/**
 * @brief  Home a single servo
 * @param  servo_id  SERVO_GRIPPER … SERVO_HATCH_B
 */
void Servo_HomeSingle(uint8_t servo_id);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
