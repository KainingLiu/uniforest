/* Hardware-free HAL boundary; production stepper.h/stepper.c stay unchanged. */
#ifndef STEPPER_TEST_HAL_H
#define STEPPER_TEST_HAL_H
#include <stdint.h>
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct {
    void *Instance;
    struct { uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload; } Init;
} TIM_HandleTypeDef;
#define GPIOI ((GPIO_TypeDef *)1)
#define GPIOH ((GPIO_TypeDef *)2)
#define GPIOD ((GPIO_TypeDef *)3)
#define GPIO_PIN_0 (1u << 0)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_14 (1u << 14)
#define GPIO_PIN_15 (1u << 15)
#define GPIO_MODE_OUTPUT_PP 0
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_HIGH 0
#define GPIO_SPEED_FREQ_LOW 0
#define TIM7 ((void *)7)
#define TIM7_IRQn 7
#define TIM_COUNTERMODE_UP 0
#define TIM_CLOCKDIVISION_DIV1 0
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0
#define TIM_IT_UPDATE 0
#define __NOP() ((void)0)
#define __HAL_RCC_GPIOI_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM7_CLK_ENABLE() ((void)0)
#define __HAL_TIM_ENABLE_IT(timer, flag) ((void)(timer), (void)(flag))
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    (void)port; (void)pin; (void)state;
}
static inline void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
    (void)port; (void)init;
}
static inline void HAL_TIM_Base_Init(TIM_HandleTypeDef *timer) { (void)timer; }
static inline void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer) { (void)timer; }
static inline void HAL_NVIC_SetPriority(int irq, int preempt, int sub) {
    (void)irq; (void)preempt; (void)sub;
}
static inline void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
#endif
