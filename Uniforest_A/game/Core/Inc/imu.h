/**
 ******************************************************************************
 * @file    imu.h
 * @brief   JY61P IMU driver (USART2, PD5=TX, PD6=RX)
 *          - Parses quaternion frames (0x59) in ISR
 *          - Extracts yaw for chassis heading-hold
 *          - 115200 bps, 8N1
 ******************************************************************************
 */

#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_OK              0
#define IMU_ERR_UART        1

uint8_t IMU_Init(void);
void    IMU_Update(void);       /* no-op: ISR-driven parsing */
float   IMU_GetYaw(void);       /* ±180°, offset by IMU_ResetYaw */
float   IMU_GetRawYaw(void);    /* absolute yaw from IMU (±180°) */
float   IMU_GetYawRate(void);   /* °/s */
void    IMU_ResetYaw(void);
uint8_t IMU_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
