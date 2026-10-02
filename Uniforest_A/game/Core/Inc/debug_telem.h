/**
 ******************************************************************************
 * @file    debug_telem.h
 * @brief   VOFA+ JustFloat telemetry output via USART3 (PD8=TX, AF7)
 *          - 115200 bps, 8N1, TX-only
 *          - Sends 18 float values per frame (72 bytes)
 *          - Call DebugTelem_Send() from main loop
 ******************************************************************************
 */

#ifndef __DEBUG_TELEM_H__
#define __DEBUG_TELEM_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize USART3 for JustFloat debug telemetry
 * @note   PB10 = USART3_TX (AF7), 921600 bps 8N1
 */
void DebugTelem_Init(void);

/**
 * @brief  Pack current telemetry as JustFloat and send on USART3
 * @note   18 floats (72 bytes): uptime, yaw, yaw_rate, 4×rpm, 4×temp,
 *         stepper_busy, 2×stepper_pos, 4×rc_ch
 *         Call at 50–100 Hz from main loop.
 */
void DebugTelem_Send(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_TELEM_H__ */
