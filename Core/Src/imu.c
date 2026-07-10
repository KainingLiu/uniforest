/**
 ******************************************************************************
 * @file    imu.c
 * @brief   JY61P IMU driver — USART2, PD5=TX, PD6=RX, direct RXNE ISR
 ******************************************************************************
 */

#include "imu.h"
#include <string.h>
#include <math.h>

#define JY61P_HEADER            0x55
#define JY61P_TYPE_QUAT         0x59

/* ====================== USART2 Pins ====================================== */

#define IMU_TX_PORT             GPIOD
#define IMU_TX_PIN              GPIO_PIN_5
#define IMU_RX_PORT             GPIOD
#define IMU_RX_PIN              GPIO_PIN_6

/* ====================== IMU State ======================================== */

static volatile float   g_yaw_deg   = 0.0f;
static volatile float   g_quat[4]   = {1.0f, 0.0f, 0.0f, 0.0f};
static volatile uint8_t g_imu_ready = 0;

static float    g_yaw_offset    = 0.0f;
static float    g_yaw_rate_ds   = 0.0f;
static uint32_t g_last_yaw_tick = 0;

UART_HandleTypeDef huart2;

/* =================== ISR Frame Parser =================================== */

void JY61P_ParseByte(uint8_t byte)
{
    static uint8_t state = 0;
    static uint8_t type;
    static uint8_t buf[8], idx;

    switch (state)
    {
    case 0:   /* wait for 0x55 */
        if (byte == JY61P_HEADER) state = 1;
        break;
    case 1:   /* record type */
        type = byte; idx = 0; state = 2;
        break;
    case 2:   /* collect 8 data bytes */
        buf[idx++] = byte;
        if (idx >= 8)
        {
            state = 0;
            if (type == JY61P_TYPE_QUAT)
            {
                int16_t q0_raw = (int16_t)((buf[1] << 8) | buf[0]);
                int16_t q1_raw = (int16_t)((buf[3] << 8) | buf[2]);
                int16_t q2_raw = (int16_t)((buf[5] << 8) | buf[4]);
                int16_t q3_raw = (int16_t)((buf[7] << 8) | buf[6]);

                float q0 = (float)q0_raw / 32768.0f;
                float q1 = (float)q1_raw / 32768.0f;
                float q2 = (float)q2_raw / 32768.0f;
                float q3 = (float)q3_raw / 32768.0f;

                g_quat[0] = q0; g_quat[1] = q1;
                g_quat[2] = q2; g_quat[3] = q3;

                /* yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2²+q3²)) */
                float yaw_new = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                       1.0f - 2.0f * (q2 * q2 + q3 * q3))
                              * 57.295779513f;

                /* differentiate for yaw rate */
                uint32_t now = HAL_GetTick();
                float dt = (float)(now - g_last_yaw_tick) / 1000.0f;
                if (dt > 0.001f && dt < 0.5f)
                {
                    float delta = yaw_new - g_yaw_deg;
                    if (delta > 180.0f)       delta -= 360.0f;
                    else if (delta < -180.0f) delta += 360.0f;
                    g_yaw_rate_ds = delta / dt;
                }
                g_yaw_deg = yaw_new;
                g_last_yaw_tick = now;
                g_imu_ready = 1;
            }
        }
        break;
    }
}

/* ====================== USART2 Init ====================================== */

static uint8_t MX_USART2_UART_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;

    gpio.Pin = IMU_TX_PIN;
    HAL_GPIO_Init(IMU_TX_PORT, &gpio);

    gpio.Pin = IMU_RX_PIN;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IMU_RX_PORT, &gpio);

    memset(&huart2, 0, sizeof(huart2));
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        return IMU_ERR_UART;

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    return IMU_OK;
}

/* ====================== Public API ======================================= */

uint8_t IMU_Init(void)
{
    g_yaw_deg       = 0.0f;
    g_yaw_offset    = 0.0f;
    g_yaw_rate_ds   = 0.0f;
    for (int i = 0; i < 4; i++) g_quat[i] = (i == 0) ? 1.0f : 0.0f;
    g_last_yaw_tick = 0;
    g_imu_ready     = 0;
    return MX_USART2_UART_Init();
}

void IMU_Update(void) { /* ISR-driven */ }

float IMU_GetYaw(void)
{
    float d = g_yaw_deg - g_yaw_offset;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

float   IMU_GetRawYaw(void)  { return g_yaw_deg; }
float   IMU_GetYawRate(void) { return g_yaw_rate_ds; }
void    IMU_ResetYaw(void)   { g_yaw_offset = g_yaw_deg; }
uint8_t IMU_IsReady(void)    { return g_imu_ready; }
