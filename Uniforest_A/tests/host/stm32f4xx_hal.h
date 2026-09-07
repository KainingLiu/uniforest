#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
#endif
