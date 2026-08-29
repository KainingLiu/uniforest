#include "suction.h"

/* PD12 pump and PD13 valve are 50 Hz PWM electronic-switch inputs, not direct
 * motor/solenoid power outputs. 0°=off, 180°=on. */
static volatile uint8_t g_pump_on;
static volatile uint8_t g_valve_on;
static volatile uint32_t g_valve_off_at;
static TIM_HandleTypeDef htim4_suction;

void Suction_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SUCTION_PUMP_PIN | SUCTION_VALVE_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(SUCTION_PUMP_PORT, &gpio);

    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4_suction.Instance = TIM4;
    htim4_suction.Init.Prescaler = SUCTION_PWM_PSC;
    htim4_suction.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4_suction.Init.Period = SUCTION_PWM_ARR;
    htim4_suction.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4_suction.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim4_suction);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.Pulse = SUCTION_PWM_OFF_PULSE;
    HAL_TIM_PWM_ConfigChannel(&htim4_suction, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim4_suction, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4_suction, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4_suction, TIM_CHANNEL_2);

    g_pump_on = 0;
    g_valve_on = 0;
    g_valve_off_at = 0;
    __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_1,
                          SUCTION_PWM_OFF_PULSE);
    __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_2,
                          SUCTION_PWM_OFF_PULSE);
}

void Suction_PumpOn(void)
{
    g_pump_on = 1;
    g_valve_on = 0;
    g_valve_off_at = 0;
    __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_1,
                          SUCTION_PWM_ON_PULSE);
}

void Suction_Release(void)
{
    /* Release is edge-triggered and deliberately non-blocking. */
    g_pump_on = 0;
    g_valve_on = 1;
    g_valve_off_at = HAL_GetTick() + SUCTION_VALVE_TIME_MS;
    __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_1,
                          SUCTION_PWM_OFF_PULSE);
    __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_2,
                          SUCTION_PWM_ON_PULSE);
}

void Suction_AllOff(void)
{
    g_pump_on = 0;
    g_valve_on = 0;
    g_valve_off_at = 0;
    __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_1,
                          SUCTION_PWM_OFF_PULSE);
    __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_2,
                          SUCTION_PWM_OFF_PULSE);
}

void Suction_Update(void)
{
    if (g_valve_off_at != 0 &&
        (int32_t)(HAL_GetTick() - g_valve_off_at) >= 0)
    {
        g_valve_off_at = 0;
        g_valve_on = 0;
        __HAL_TIM_SET_COMPARE(&htim4_suction, TIM_CHANNEL_2,
                              SUCTION_PWM_OFF_PULSE);
    }
}
