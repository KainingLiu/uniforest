/* Host-only HAL: executes production motion state machines without hardware. */
#ifndef TEST_STM32_HAL_H
#define TEST_STM32_HAL_H
#include <stdint.h>
#include <string.h>

typedef struct { unsigned unused; } GPIO_TypeDef;
typedef struct { unsigned Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct {
    void *Instance;
    struct { unsigned Prescaler, CounterMode, Period, ClockDivision,
                      AutoReloadPreload; } Init;
} TIM_HandleTypeDef;
typedef struct { void *Instance; } CAN_HandleTypeDef;
typedef struct { void *Instance; struct { unsigned BaudRate, WordLength, StopBits,
    Parity, Mode, HwFlowCtl, OverSampling; } Init; } UART_HandleTypeDef;
typedef struct { unsigned StdId, IDE, RTR, DLC; } CAN_TxHeaderTypeDef;
typedef struct { unsigned StdId; } CAN_RxHeaderTypeDef;
typedef struct { unsigned FilterIdHigh, FilterIdLow, FilterMaskIdHigh,
    FilterMaskIdLow, FilterFIFOAssignment, FilterBank, FilterMode, FilterScale,
    FilterActivation, SlaveStartFilterBank; } CAN_FilterTypeDef;

static GPIO_TypeDef host_gpio[5];
#define GPIOI (&host_gpio[0])
#define GPIOH (&host_gpio[1])
#define GPIOD (&host_gpio[2])
#define GPIOE (&host_gpio[3])
#define GPIOF (&host_gpio[4])
#define GPIO_PIN_0 (1u << 0)
#define GPIO_PIN_2 (1u << 2)
#define GPIO_PIN_3 (1u << 3)
#define GPIO_PIN_4 (1u << 4)
#define GPIO_PIN_5 (1u << 5)
#define GPIO_PIN_7 (1u << 7)
#define GPIO_PIN_8 (1u << 8)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_14 (1u << 14)
#define GPIO_PIN_15 (1u << 15)
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_MODE_OUTPUT_PP 0
#define GPIO_NOPULL 0
#define GPIO_PULLUP 0
#define GPIO_MODE_AF_PP 0
#define GPIO_AF8_UART7 0
#define UART7 ((void *)7)
#define UART7_IRQn 7
#define UART_WORDLENGTH_8B 0
#define UART_STOPBITS_1 0
#define UART_PARITY_NONE 0
#define UART_MODE_TX_RX 0
#define UART_HWCONTROL_NONE 0
#define UART_OVERSAMPLING_8 0
#define UART_IT_RXNE 0
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_UART7_CLK_ENABLE() ((void)0)
#define __HAL_UART_ENABLE_IT(u, i) ((void)0)
#define GPIO_SPEED_FREQ_HIGH 0
#define GPIO_SPEED_FREQ_LOW 0
#define TIM7 ((void *)7)
#define TIM7_IRQn 7
#define TIM_COUNTERMODE_UP 0
#define TIM_CLOCKDIVISION_DIV1 0
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0
#define TIM_IT_UPDATE 0
#define CAN1 ((void *)1)
#define CAN1_RX0_IRQn 1
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
#define CAN_RX_FIFO0 0
#define CAN_FILTER_FIFO0 0
#define CAN_FILTERMODE_IDMASK 0
#define CAN_FILTERSCALE_32BIT 0
#define CAN_IT_RX_FIFO0_MSG_PENDING 0
#define ENABLE 1
#define HAL_OK 0
#define __HAL_RCC_GPIOI_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM7_CLK_ENABLE() ((void)0)
#define __HAL_TIM_ENABLE_IT(timer, flag) ((void)0)
#define __NOP() ((void)0)

static uint32_t host_tick, host_irq;
static uint8_t host_last_can[8];
static uint32_t host_can_irq;
static CAN_HandleTypeDef hcan1 = {CAN1};
static uint32_t HAL_GetTick(void) { return host_tick; }
static void HAL_Delay(uint32_t ms) { host_tick += ms; }
static uint8_t host_uart_frames[256][86];
static unsigned host_uart_lengths[256], host_uart_count;
static int HAL_UART_Init(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
static int HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *data, unsigned n, unsigned timeout)
{
    (void)u; (void)timeout;
    unsigned i=host_uart_count++ % 256;
    host_uart_lengths[i]=n;
    memcpy(host_uart_frames[i],data,n);
    return HAL_OK;
}
static uint32_t __get_PRIMASK(void) { return host_irq; }
static void __disable_irq(void) { host_irq = 1; }
static void __set_PRIMASK(uint32_t value) { host_irq = value; }
static void HAL_GPIO_WritePin(GPIO_TypeDef *p, unsigned pin, unsigned value)
    { (void)p; (void)pin; (void)value; }
static void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i)
    { (void)p; (void)i; }
static void HAL_TIM_Base_Init(TIM_HandleTypeDef *t) { (void)t; }
static void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *t) { (void)t; }
static void HAL_NVIC_SetPriority(unsigned i, unsigned p, unsigned s)
    { (void)i; (void)p; (void)s; }
static void HAL_NVIC_EnableIRQ(unsigned i) { (void)i; }
static int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *c, CAN_TxHeaderTypeDef *t,
                               uint8_t *d, uint32_t *m)
    { (void)c; (void)t; *m = 0; memcpy(host_last_can, d, 8);
      host_can_irq = host_irq; return HAL_OK; }
static unsigned HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *c, unsigned f)
    { (void)c; (void)f; return 0; }
static int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *c, unsigned f,
                               CAN_RxHeaderTypeDef *r, uint8_t *d)
    { (void)c; (void)f; (void)r; (void)d; return 1; }
static void HAL_CAN_Start(CAN_HandleTypeDef *c) { (void)c; }
static void HAL_CAN_ConfigFilter(CAN_HandleTypeDef *c, CAN_FilterTypeDef *f)
    { (void)c; (void)f; }
static void HAL_CAN_ActivateNotification(CAN_HandleTypeDef *c, unsigned n)
    { (void)c; (void)n; }
#endif
