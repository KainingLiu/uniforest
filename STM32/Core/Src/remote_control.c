/**
 ******************************************************************************
 * @file    remote_control.c
 * @brief   SBUS (DBUS) remote control — receiver, parser, RC main loop
 *
 *  SBUS frame: 0x0F + 22 data bytes + flags + 0x00
 *  100 kbps, 8E2, inverted logic (CR2.RXINV)
 *
 *  Channel mapping:
 *    CH1 (idx 0) → vy  (left/right lateral)
 *    CH2 (idx 1) → vx  (forward/backward)
 *    CH4 (idx 3) → wz  (left/right rotation)
 *
 *  USART1 on PB7 (AF7), RX-only, interrupt-driven byte-at-a-time parsing.
 ******************************************************************************
 */

#include "remote_control.h"
#include "motor3508.h"
#include "imu.h"
#include "actions.h"
#include <string.h>

/* ==================== SBUS (DBUS) Protocol =================================== */

#define SBUS_BAUDRATE             100000   /* 100 kbps */
#define SBUS_FRAME_SIZE           25       /* 0x0F + 22 data + flags + 0x00 */
#define SBUS_CHANNEL_COUNT        16
#define SBUS_CENTER               1024     /* stick center value (11-bit) */
#define SBUS_DEAD_ZONE            80       /* ~8% dead zone around center (matches old PWM 10%) */
#define SBUS_TIMEOUT_MS           100      /* signal loss if no valid frame for 100ms */

/* Channel mapping: 0-based SBUS channels → control axes */
#define SBUS_CH_VY                0       /* CH1: left/right (lateral) */
#define SBUS_CH_VX                1       /* CH2: forward/backward */
#define SBUS_CH_WZ                3       /* CH4: left/right rotation */
#define SBUS_CH_HATCH             4       /* CH5: hatch switch */

#define SBUS_HATCH_THRESHOLD      600     /* ~1250 us — below → open, above → close */

/* ---- SBUS Frame Parser State ---- */

typedef enum {
    SBUS_WAIT_HEADER = 0,  /* waiting for start byte 0x0F */
    SBUS_DATA,             /* collecting 22 channel data bytes */
    SBUS_FLAGS,            /* collecting flags byte (byte 23) */
    SBUS_END               /* waiting for end byte 0x00 */
} SbusState_t;

/* USART1 handle — non-static so ISR in stm32f4xx_it.c can extern it */
UART_HandleTypeDef huart1;

/* Parser internal state */
static SbusState_t  g_sbus_state     = SBUS_WAIT_HEADER;
static uint8_t      g_sbus_buf[SBUS_FRAME_SIZE];
static uint8_t      g_sbus_idx       = 0;

/* Latest decoded frame */
static uint16_t     g_sbus_channels[SBUS_CHANNEL_COUNT];
static uint32_t     g_sbus_last_tick  = 0;
static uint8_t      g_sbus_valid      = 0;

/* ---- SBUS Frame Unpacker ---- */

/**
 * @brief  Unpack SBUS 22-byte payload into 16 channel values (0-2047)
 * @note   Each channel is 11 bits, packed MSB-first across 22 bytes.
 *         Overshoots by 1 byte for CH15 — the flags byte sits at data[22]
 *         and its bits are masked out at the MSB end of the 0x07FF mask.
 */
static void SBUS_UnpackFrames(const uint8_t data[22])
{
    uint16_t bit_pos = 0;
    for (int ch = 0; ch < SBUS_CHANNEL_COUNT; ch++)
    {
        uint8_t  byte_idx    = bit_pos / 8;
        uint8_t  bit_offset  = bit_pos % 8;
        uint32_t raw         = (uint32_t)data[byte_idx]
                             | ((uint32_t)data[byte_idx + 1] << 8)
                             | ((uint32_t)data[byte_idx + 2] << 16);
        g_sbus_channels[ch]  = (uint16_t)((raw >> bit_offset) & 0x07FF);
        bit_pos += 11;
    }
}

/* ---- SBUS Parser State Machine ---- */

/**
 * @brief  Feed one byte into the SBUS frame parser (called from USART1 ISR)
 * @note   Syncs on 0x0F start byte, validates 0x00 end byte.
 *         On corrupted end byte, immediately re-checks if it's a 0x0F for fast re-sync.
 */
void SBUS_ParseByte(uint8_t byte)
{
    switch (g_sbus_state)
    {
    case SBUS_WAIT_HEADER:
        if (byte == 0x0F)
        {
            g_sbus_buf[0] = byte;
            g_sbus_idx = 1;
            g_sbus_state = SBUS_DATA;
        }
        break;

    case SBUS_DATA:
        g_sbus_buf[g_sbus_idx++] = byte;
        if (g_sbus_idx >= 23)  /* header + 22 data bytes collected */
        {
            g_sbus_state = SBUS_FLAGS;
        }
        break;

    case SBUS_FLAGS:
        g_sbus_buf[23] = byte;   /* flags byte */
        g_sbus_state = SBUS_END;
        break;

    case SBUS_END:
        if (byte == 0x00)
        {
            /* Valid frame complete */
            g_sbus_buf[24] = byte;
            SBUS_UnpackFrames(&g_sbus_buf[1]);  /* skip 0x0F start byte */
            g_sbus_last_tick = HAL_GetTick();
            g_sbus_valid = 1;
        }
        /* Frame corrupted — re-check if this byte starts a new header */
        g_sbus_state = SBUS_WAIT_HEADER;
        if (byte == 0x0F)
        {
            g_sbus_buf[0] = byte;
            g_sbus_idx = 1;
            g_sbus_state = SBUS_DATA;
        }
        break;
    }
}

/**
 * @brief  Reset SBUS parser state (called on UART framing errors)
 */
void SBUS_ResetParser(void)
{
    g_sbus_state = SBUS_WAIT_HEADER;
    g_sbus_idx = 0;
}

/* ---- SBUS Channel Reader ---- */

/**
 * @brief  Check if SBUS signal is currently valid (fresh frame within timeout)
 */
uint8_t SBUS_IsValid(void)
{
    if (!g_sbus_valid) return 0;
    if ((HAL_GetTick() - g_sbus_last_tick) > SBUS_TIMEOUT_MS)
    {
        g_sbus_valid = 0;
        return 0;
    }
    return 1;
}

/**
 * @brief  Read raw SBUS channel value (0-2047), or 0 if signal lost
 */
uint16_t SBUS_GetChannel(uint8_t ch)
{
    if (ch >= SBUS_CHANNEL_COUNT || !SBUS_IsValid()) return 0;
    return g_sbus_channels[ch];
}

/**
 * @brief  Map SBUS channel value to normalised velocity offset [-1.0, +1.0]
 * @note   Center = 1024, dead zone = ±40. Excludes dead zone from denominator
 *         so stick output ramps 0→1 smoothly from dead-zone edge to endpoint.
 */
static float SBUS_ChannelToOffset(uint16_t ch_val)
{
    int16_t diff = (int16_t)ch_val - (int16_t)SBUS_CENTER;   /* ±1024 range */

    /* Dead zone (~10% of operating range, matches old PWM spec) */
    if (diff > -SBUS_DEAD_ZONE && diff < SBUS_DEAD_ZONE)
        return 0.0f;

    /* Clamp and normalise — same style as old PWM:
     *   dead zone is a "hole", full stick range maps to ±1 */
    if (diff > 800)  diff = 800;
    if (diff < -800) diff = -800;

    return (float)diff / 800.0f;
}

/* ---- Initialization ---- */

/**
 * @brief  Initialize SBUS (DBUS) receiver via USART1
 * @note   Pin: PB7 = USART1_RX (AF7)
 *         100,000 bps, 8E2 (8 data + even parity + 2 stop bits)
 *         SBUS signal is logic-level INVERTED → CR2.RXINV handles it in h/w
 */
void Remote_Control_Init(void)
{
    /* ---- 1. Clock enable ---- */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* ---- 2. GPIO: PB7 = USART1_RX, AF7 ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    gpio.Pin       = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* ---- 3. USART1: 100000 bps, 8E2 ---- */
    memset(&huart1, 0, sizeof(huart1));
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = SBUS_BAUDRATE;
    huart1.Init.WordLength   = UART_WORDLENGTH_9B;    /* 8 data + parity */
    huart1.Init.StopBits     = UART_STOPBITS_2;
    huart1.Init.Parity       = UART_PARITY_EVEN;
    huart1.Init.Mode         = UART_MODE_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_8;
    HAL_UART_Init(&huart1);

    /* ---- 4. Invert RX signal in hardware (SBUS is inverted) ---- */
    huart1.Instance->CR2 |= (1UL << 19);  /* USART_CR2_RXINV (bit 19) */

    /* ---- 5. Enable RXNE interrupt ---- */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    /* ---- 6. NVIC: USART1 IRQ, priority 2 (same as USART2/IMU) ---- */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* ---- 7. Init parser state ---- */
    g_sbus_valid     = 0;
    g_sbus_state     = SBUS_WAIT_HEADER;
    g_sbus_idx       = 0;
    memset(g_sbus_channels, 0, sizeof(g_sbus_channels));
}

/* ---- Main Remote Control Loop ---- */

/**
 * @brief  RC remote control with IMU yaw heading-hold
 * @note   SBUS channels: CH2→vx(fwd+), CH1→vy(right+), CH4→wz(CW+)
 *         Locks heading on entry. IMU PID corrects drift when not rotating.
 *         Re-locks heading when rotation stick is released.
 *         Exits on PB2 key press; signal loss → all axes zeroed.
 */
void Remote_Control(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);  /* green ON */

    PID_t yaw_pid;
    PID_Init(&yaw_pid, RC_YAW_KP, RC_YAW_KI, 0.0f,
             RC_YAW_MAX_RPM, RC_YAW_MAX_RPM);

    float   yaw_ref = IMU_GetRawYaw();   /* target heading, integrated from stick */
    uint8_t hatch_was_open = 0;

    while (1)
    {
        Motor3508_RxPoll();
        IMU_Update();

        /* Read SBUS channels */
        float vx_norm = SBUS_ChannelToOffset(SBUS_GetChannel(SBUS_CH_VX));
        float vy_norm = SBUS_ChannelToOffset(SBUS_GetChannel(SBUS_CH_VY));
        float wz_norm = SBUS_ChannelToOffset(SBUS_GetChannel(SBUS_CH_WZ));

        /* Signal loss safety: zero all axes if no valid SBUS frame */
        if (!SBUS_IsValid())
        {
            vx_norm = 0.0f;
            vy_norm = 0.0f;
            wz_norm = 0.0f;
        }

        /* ---- Hatch control: CH5 < 1250us → open, else close ---- */
        if (SBUS_IsValid())
        {
            uint16_t ch5 = SBUS_GetChannel(SBUS_CH_HATCH);
            uint8_t  open = (ch5 < SBUS_HATCH_THRESHOLD);
            if (open && !hatch_was_open)
                Actions_HatchOpen();
            else if (!open && hatch_was_open)
                Actions_HatchClose();
            hatch_was_open = open;
        }

        /* ---- CH6 mode switch: break out to main loop if CH6 leaves centre ---- */
        if (SBUS_IsValid())
        {
            uint16_t ch6 = SBUS_GetChannel(5);
            if (ch6 > SBUS_CH6_HIGH || ch6 < SBUS_CH6_LOW)
                break;
        }

        float vx_cm_s = vx_norm * RC_MAX_LINEAR_SPEED_CM_S;
        float vy_cm_s = vy_norm * RC_MAX_LINEAR_SPEED_CM_S;

        /* Integrate rotation stick → target heading, then let yaw PID steer */
        yaw_ref -= wz_norm * RC_MAX_ANGULAR_SPEED_DEG_S * PID_DT;
        while (yaw_ref >= 360.0f)  yaw_ref -= 360.0f;
        while (yaw_ref < 0.0f)     yaw_ref += 360.0f;

        /* Yaw PID: steer toward integrated target heading */
        float yaw_err = yaw_ref - IMU_GetRawYaw();
        while (yaw_err > 180.0f)  yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;
        float yaw_corr_rpm = PID_Compute(&yaw_pid, 0.0f, yaw_err, PID_DT);

        /* Mecanum kinematics: translation only, rotation handled by yaw PID */
        float rpm[4];
        Motor3508_MecanumRPM(vx_cm_s, vy_cm_s, 0.0f, rpm);
        rpm[0] += yaw_corr_rpm;
        rpm[1] += yaw_corr_rpm;
        rpm[2] += yaw_corr_rpm;
        rpm[3] += yaw_corr_rpm;

        for (int i = 0; i < M3508_COUNT; i++)
            Motor3508_SetSpeed(i, (int16_t)rpm[i]);

        Motor3508_UpdateAllSpeedPID(PID_DT);

        /* Exit on key press */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)
        {
            HAL_Delay(30);
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)
            {
                while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)
                    HAL_Delay(10);
                break;
            }
        }

        HAL_Delay(PID_DT * 1000.0f);
    }

    Motor3508_StopAll();
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);   /* green OFF */
}
