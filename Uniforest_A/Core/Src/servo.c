/**
 ******************************************************************************
 * @file    servo.c
 * @brief   Servo PWM control implementation
 *          TIM2 CH2/3/4 (PA1/PA2/PA3) + TIM5 CH1 (PA0) @ 50 Hz
 ******************************************************************************
 */

#include "servo.h"

/* ============================ Private Variables =========================== */

/* Timer handles */
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim5;

/* Per-servo configuration table (indexed by servo ID) */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    uint8_t            home_angle;
} ServoEntry_t;

static const ServoEntry_t servo_table[SERVO_COUNT] = {
    [SERVO_GRIPPER]   = { &htim2, SERVO0_CHANNEL, SERVO1_HOME   },
    [SERVO_ARM_FRONT] = { &htim2, SERVO1_CHANNEL, SERVO2_HOME   },
    [SERVO_HATCH_A]   = { &htim2, SERVO2_CHANNEL, SERVO3_HOME   },
    [SERVO_HATCH_B]   = { &htim5, SERVO3_CHANNEL, SERVO4_HOME   },
};

/* ============================ Implementation ============================== */

/**
 * @brief  Initialize all 4 servo PWM outputs
 * @note   Configures GPIO alternate functions:
 *           PA1 → AF1 (TIM2_CH2)
 *           PA2 → AF1 (TIM2_CH3)
 *           PA3 → AF1 (TIM2_CH4)
 *           PA0 → AF2 (TIM5_CH1)
 *         Timer clock: APB1 = 45 MHz → timer clock = 90 MHz
 *         PSC = 179  → 90 MHz / 180 = 500 kHz
 *         ARR = 9999 → 500 kHz / 10000 = 50 Hz
 */
void Servo_Init(void)
{
    /* ---- Configure GPIO alternate functions ---- */

    /* PA1, PA2, PA3 → AF1 (TIM2); PA0 → AF2 (TIM5) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode       = GPIO_MODE_AF_PP;
    gpio.Pull       = GPIO_NOPULL;
    gpio.Speed      = GPIO_SPEED_FREQ_LOW;

    /* TIM2 pins: PA1/PA2/PA3 → AF1 */
    gpio.Alternate  = GPIO_AF1_TIM2;
    gpio.Pin        = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* TIM5 pin: PA0 → AF2 */
    gpio.Alternate  = GPIO_AF2_TIM5;
    gpio.Pin        = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---- TIM2: channels 2, 3, 4 ---- */
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = SERVO_TIM_PSC;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = SERVO_TIM_ARR;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.Pulse      = SERVO_PULSE_MID;

    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    /* ---- TIM5: channel 1 (PA0) ---- */
    __HAL_RCC_TIM5_CLK_ENABLE();
    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = SERVO_TIM_PSC;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = SERVO_TIM_ARR;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim5);

    oc.Pulse = SERVO_PULSE_MID;
    HAL_TIM_PWM_ConfigChannel(&htim5, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

    /* Home all servos to idle positions immediately */
    Servo_HomeAll();
}

/**
 * @brief  Set a servo to a specific angle
 * @param  servo_id   SERVO_GRIPPER (0) … SERVO_HATCH_B (3)
 * @param  angle_deg  0–180 degrees, clamped
 * @note   Linear mapping: pulse = 250 + (angle * 1000) / 180
 */
void Servo_SetAngle(uint8_t servo_id, uint8_t angle_deg)
{
    if (servo_id >= SERVO_COUNT) return;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;

    /* Linear mapping: 0°→250, 180°→1250 */
    uint32_t pulse = SERVO_PULSE_MIN
                   + ((uint32_t)angle_deg * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180;

    const ServoEntry_t *s = &servo_table[servo_id];
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, pulse);
}

/**
 * @brief  Set all 4 servos to their home (idle) positions
 */
void Servo_HomeAll(void)
{
    Servo_SetAngle(SERVO_GRIPPER,   SERVO1_HOME);   /* 81°  gripper open */
    Servo_SetAngle(SERVO_ARM_FRONT, SERVO2_HOME);   /* 90°  arm down */
    Servo_SetAngle(SERVO_HATCH_A,   SERVO3_HOME);   /* 63°  hatch A closed */
    Servo_SetAngle(SERVO_HATCH_B,   SERVO4_HOME);   /* 117° hatch B closed */
}

/**
 * @brief  Home a single servo to its default position
 */
void Servo_HomeSingle(uint8_t servo_id)
{
    if (servo_id >= SERVO_COUNT) return;
    Servo_SetAngle(servo_id, servo_table[servo_id].home_angle);
}
