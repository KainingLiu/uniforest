#ifndef __SUCTION_H__
#define __SUCTION_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Hardware constraint: both outputs drive PWM electronic switches.
 * Never replace the PWM waveform with a continuous GPIO high level.
 * PD12/PD13 and active-high behavior match the suction test program.
 */
#define SUCTION_PUMP_PORT       GPIOD
#define SUCTION_PUMP_PIN        GPIO_PIN_12
#define SUCTION_VALVE_PORT      GPIOD
#define SUCTION_VALVE_PIN       GPIO_PIN_13
#define SUCTION_VALVE_TIME_MS   1000u

/* TIM4: 50 Hz PWM, same 0.5-2.5 ms mapping as the servo outputs. */
#define SUCTION_PWM_PSC          179u
#define SUCTION_PWM_ARR          9999u
#define SUCTION_PWM_OFF_PULSE    250u
#define SUCTION_PWM_ON_PULSE     1250u

void Suction_Init(void);
void Suction_PumpOn(void);
void Suction_Release(void);
void Suction_AllOff(void);
void Suction_Update(void);

#ifdef __cplusplus
}
#endif

#endif
