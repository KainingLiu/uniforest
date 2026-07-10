/**
 ******************************************************************************
 * @file    remote_control.h
 * @brief   SBUS (DBUS) remote control receiver and dispatch
 ******************************************************************************
 */

#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- RC Heading-Hold Yaw PID ---------- */

#define RC_YAW_KP                   100.0f   /* 100 RPM/° */
#define RC_YAW_KI                     0.8f   /* low integral → no overshoot */
#define RC_YAW_MAX_RPM              3000.0f  /* max yaw correction */

/* ---------- Chassis Speed Limits ---------- */

#define RC_MAX_LINEAR_SPEED_CM_S     250.0f  /* max fwd/lateral speed (2.5 m/s) */
#define RC_MAX_ANGULAR_SPEED_DEG_S   300.0f  /* max angular speed */

/* ---------- SBUS Control Thresholds ---------- */

#define SBUS_ARM_THRESHOLD          1600    /* ~1900 us — CH1/2/4 must exceed to arm */
#define SBUS_CH6_HIGH               1750    /* ~1975 us — must be definitively high to trigger Grap */
#define SBUS_CH6_LOW                 450    /* ~1160 us — must be definitively low to trigger Build */
#define SBUS_CH6_ARM_LOW             900    /* ~1442 us — CH6 must be above to arm */
#define SBUS_CH6_ARM_HIGH           1150    /* ~1600 us — CH6 must be below to arm */

/* ---------- SBUS (DBUS) Callbacks (used by ISR) ---------- */

extern UART_HandleTypeDef huart1;            /* USART1 peripheral handle */
void SBUS_ParseByte(uint8_t byte);           /* ISR byte feeder */
void SBUS_ResetParser(void);                 /* ISR error recovery */

/* ---------- SBUS Channel Reader (used by main.c) ---------- */

uint8_t  SBUS_IsValid(void);                 /* signal present? */
uint16_t SBUS_GetChannel(uint8_t ch);        /* raw channel value (0-2047) */

/* ---------- Public API ---------- */

void Remote_Control_Init(void);              /* init USART1 + SBUS parser */
void Remote_Control(void);                   /* main RC loop (blocking) */

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_CONTROL_H__ */
