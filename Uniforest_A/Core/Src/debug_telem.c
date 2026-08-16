/**
 ******************************************************************************
 * @file    debug_telem.c
 * @brief   VOFA+ JustFloat telemetry — USART3 (PB10=TX, AF7)
 *
 *  JustFloat: raw little-endian float32 array, no header, no footer.
 *  VOFA+ auto-detects frame size (72 bytes = 18 floats).
 *
 *  Channel mapping:
 *    [0]  uptime_s       system uptime (seconds)
 *    [1]  yaw_deg        IMU yaw angle (°)
 *    [2]  yaw_rate       IMU yaw rate (°/s)
 *    [3]  rpm_TR         TR motor speed (RPM)
 *    [4]  rpm_TL         TL motor speed (RPM)
 *    [5]  rpm_BL         BL motor speed (RPM)
 *    [6]  rpm_BR         BR motor speed (RPM)
 *    [7]  temp_TR        TR motor temperature (°C)
 *    [8]  temp_TL        TL motor temperature (°C)
 *    [9]  temp_BL        BL motor temperature (°C)
 *    [10] temp_BR        BR motor temperature (°C)
 *    [11] stepper_busy   0=idle, 1=H busy, 2=V busy, 3=both
 *    [12] stepper_H      horizontal stepper position (steps)
 *    [13] stepper_V      vertical stepper position (steps)
 *    [14] rc_CH1         RC channel 1 (0–2047)
 *    [15] rc_CH2         RC channel 2
 *    [16] rc_CH3         RC channel 3
 *    [17] rc_CH4         RC channel 4
 ******************************************************************************
 */

#include "debug_telem.h"
#include "motor3508.h"
#include "imu.h"
#include "stepper.h"
#include "remote_control.h"
#include <string.h>

/* ============================ Constants ==================================== */

#define USART3_BAUDRATE         115200u
#define TELEM_FLOAT_COUNT       18u
#define TELEM_FRAME_SIZE        (TELEM_FLOAT_COUNT * 4u)  /* 72 bytes */

/* ============================ UART Handle ================================== */

static UART_HandleTypeDef huart3_debug;

/* ======================== JustFloat Helper ================================= */

/**
 * @brief  Store a float32 as little-endian bytes
 */
static inline void put_float_le(uint8_t *buf, uint16_t *pos, float val)
{
    /* Use union for safe type-punning (avoids unaligned access warnings) */
    union { float f; uint32_t u; } u;
    u.f = val;
    buf[(*pos)++] = (uint8_t)(u.u);
    buf[(*pos)++] = (uint8_t)(u.u >> 8);
    buf[(*pos)++] = (uint8_t)(u.u >> 16);
    buf[(*pos)++] = (uint8_t)(u.u >> 24);
}

/* ======================== Initialization =================================== */

void DebugTelem_Init(void)
{
    /* ---- Clocks ---- */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* ---- PD8 = USART3_TX (AF7) ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    gpio.Pin       = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* ---- USART3: 921600-8N1, TX only ---- */
    memset(&huart3_debug, 0, sizeof(huart3_debug));
    huart3_debug.Instance          = USART3;
    huart3_debug.Init.BaudRate     = USART3_BAUDRATE;
    huart3_debug.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3_debug.Init.StopBits     = UART_STOPBITS_1;
    huart3_debug.Init.Parity       = UART_PARITY_NONE;
    huart3_debug.Init.Mode         = UART_MODE_TX;
    huart3_debug.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3_debug.Init.OverSampling = UART_OVERSAMPLING_8;
    HAL_UART_Init(&huart3_debug);
}

/* ======================== JustFloat Sender ================================= */

void DebugTelem_Send(void)
{
    uint8_t buf[TELEM_FRAME_SIZE];
    uint16_t pos = 0;

    /* ---- Motor telemetry ---- */
    uint16_t angle;
    int16_t  rpm, torque;
    uint8_t  temp;

    for (int i = 0; i < 4; i++)
    {
        Motor3508_GetFeedback(i, &angle, &rpm, &torque, &temp);
    }

    /* ---- Pack floats ---- */

    /* [0] Uptime (seconds) */
    put_float_le(buf, &pos, (float)HAL_GetTick() / 1000.0f);

    /* [1-2] IMU */
    put_float_le(buf, &pos, IMU_GetYaw());
    put_float_le(buf, &pos, IMU_GetYawRate());

    /* [3-6] Motor RPMs (TR, TL, BL, BR) */
    Motor3508_GetFeedback(0, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)rpm);
    Motor3508_GetFeedback(1, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)rpm);
    Motor3508_GetFeedback(2, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)rpm);
    Motor3508_GetFeedback(3, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)rpm);

    /* [7-10] Motor temperatures */
    Motor3508_GetFeedback(0, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)temp);
    Motor3508_GetFeedback(1, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)temp);
    Motor3508_GetFeedback(2, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)temp);
    Motor3508_GetFeedback(3, &angle, &rpm, &torque, &temp);
    put_float_le(buf, &pos, (float)temp);

    /* [11] Stepper busy flags */
    put_float_le(buf, &pos, (float)((Stepper_IsBusy(0) ? 1 : 0)
                                  | (Stepper_IsBusy(1) ? 2 : 0)));

    /* [12-13] Stepper positions */
    put_float_le(buf, &pos, (float)Stepper_GetPosition(0));
    put_float_le(buf, &pos, (float)Stepper_GetPosition(1));

    /* [14-17] RC channels 1-4 */
    put_float_le(buf, &pos, (float)SBUS_GetChannel(0));
    put_float_le(buf, &pos, (float)SBUS_GetChannel(1));
    put_float_le(buf, &pos, (float)SBUS_GetChannel(2));
    put_float_le(buf, &pos, (float)SBUS_GetChannel(3));

    /* Send frame */
    HAL_UART_Transmit(&huart3_debug, buf, TELEM_FRAME_SIZE, 2);
}
