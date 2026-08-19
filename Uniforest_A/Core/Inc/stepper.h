/**
 ******************************************************************************
 * @file    stepper.h
 * @brief   Non-blocking stepper motor control via TIM7 interrupt (100 kHz)
 *          Two TB6600-driven steppers with independent trapezoidal profiles.
 *          Dual-motor coordinated moves (overlap / overlap-with-dir-change).
 *
 *  Pin assignments:
 *  ┌──────────────┬──────────┬────────┬────────┐
 *  │    Motor     │   PUL    │  DIR   │  ENA   │
 *  ├──────────────┼──────────┼────────┼────────┤
 *  │ Stepper1 (H) │ A = PI0  │B = PH12│C = PH11│
 *  │ Stepper2 (V) │ D = PH10 │E = PD15│F = PD14│
 *  ├──────────────┼──────────┼────────┼────────┤
 *  │ LED1         │ G = PD13 │        │        │
 *  │ LED2         │ H = PD12 │        │        │
 *  └──────────────┴──────────┴────────┴────────┘
 *
 *  TIM7: 100 kHz (10 µs tick) → ISR → Stepper_Tick()
 *  Mechanical: 400 steps/rev (2× microstepping), 10 mm lead → 400 steps/cm
 ******************************************************************************
 */

#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stm32f4xx_hal.h"

/* TIM7 handle — used by ISR in stm32f4xx_it.c */
extern TIM_HandleTypeDef htim7_stepper;

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= Motor IDs ====================================== */

#define STEPPER_HORIZ               0       /* Stepper 1: horizontal */
#define STEPPER_VERT                1       /* Stepper 2: vertical */
#define STEPPER_COUNT               2

/* ================= Direction & Enable (TB6600 common-cathode) ============= */

#define STEP_DIR_FORWARD            0       /* LOW → forward */
#define STEP_DIR_REVERSE            1       /* HIGH → reverse */
#define STEP_ENA_ON                 0       /* LOW → enabled (energized) */
#define STEP_ENA_OFF                1       /* HIGH → disabled (free) */

/* ======================= Speed Profile Parameters ========================= */

#define STEP_START_DELAY_US         1000    /* actual start ~417 Hz after safety scale */
#define STEP_TARGET_DELAY_US        100     /* actual cruise ~4.17 kHz after safety scale */
#define STEP_ACCEL_STEPS            400     /* accel / decel ramp length */

/*
 * TIM7 makes the 0810 timing more exact than the old busy-loop implementation.
 * Apply a small 6/5 half-cycle margin to reduce lost-step risk without making
 * the replicated action sequence significantly slower. Keep the wire/API
 * parameters unchanged and compensate at the timer conversion boundary.
 */
#define STEP_DELAY_SCALE_NUM        6u
#define STEP_DELAY_SCALE_DEN        5u

/* ======================= Steps per Centimeter ============================= */
/*  400 steps/rev  ÷  10 mm/rev  =  40 steps/mm  =  400 steps/cm              */

#define STEPS_PER_CM                400

/* ====================== delay_us Calibration ============================== */
/*  180 MHz SYSCLK ÷ 12 (loop overhead) ≈ 15                                  */

#define DELAY_US_COEFF              15

/* =========================== Pin Definitions ============================== */

/* ---- Stepper 1 (Horizontal) ---- */
#define STEP1_PUL_PORT              GPIOI
#define STEP1_PUL_PIN               GPIO_PIN_0              /* A = PI0 */
#define STEP1_DIR_PORT              GPIOH
#define STEP1_DIR_PIN               GPIO_PIN_12             /* B = PH12 */
#define STEP1_ENA_PORT              GPIOH
#define STEP1_ENA_PIN               GPIO_PIN_11             /* C = PH11 */

/* ---- Stepper 2 (Vertical) ---- */
#define STEP2_PUL_PORT              GPIOH
#define STEP2_PUL_PIN               GPIO_PIN_10             /* D = PH10 */
#define STEP2_DIR_PORT              GPIOD
#define STEP2_DIR_PIN               GPIO_PIN_15             /* E = PD15 */
#define STEP2_ENA_PORT              GPIOD
#define STEP2_ENA_PIN               GPIO_PIN_14             /* F = PD14 */

/* ---- LED Indicators ---- */
#define LED_STEPPER1_PORT           GPIOD
#define LED_STEPPER1_PIN            GPIO_PIN_13             /* G = PD13 */
#define LED_STEPPER2_PORT           GPIOD
#define LED_STEPPER2_PIN            GPIO_PIN_12             /* H = PD12 */

/* ========================== Data Structures =============================== */

/**
 * @brief Per-stepper GPIO pin bundle
 */
typedef struct {
    GPIO_TypeDef *pul_port;
    uint16_t      pul_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    GPIO_TypeDef *ena_port;
    uint16_t      ena_pin;
} StepperPins_t;

/* ========================== Public API ==================================== */

/* ---------- Initialization ---------- */
void Stepper_Init(void);

/* ---------- Low-Level Control ---------- */
void Stepper_SetDir(uint8_t motor, uint8_t dir);
void Stepper_Enable(uint8_t motor, uint8_t ena);

/**
 * @brief  NOP-based microsecond delay (kept for compatibility)
 */
void delay_us(uint32_t us);

/* ---------- Default Speed Parameters ---------- */

/**
 * @brief  Set default trapezoidal speed parameters for subsequent moves
 * @param  start_delay   initial half-cycle delay (µs)
 * @param  target_delay  minimum half-cycle delay at cruise (µs)
 * @param  accel_steps   number of accel/decel steps
 * @note   Affects future Stepper_StartMove() calls.
 *          Individual move functions can override these.
 */
void Stepper_SetParams(uint16_t start_delay, uint16_t target_delay,
                       uint16_t accel_steps);

/* ---------- Non-Blocking Single Move ---------- */

/**
 * @brief  Launch a trapezoidal move for one stepper (non-blocking)
 * @param  motor        STEPPER_HORIZ or STEPPER_VERT
 * @param  dir          STEP_DIR_FORWARD or STEP_DIR_REVERSE
 * @param  steps        total step count
 * @param  start_delay  µs half-cycle delay at ramp start (0 = use defaults)
 * @param  target_delay µs half-cycle delay at cruise (0 = use defaults)
 * @param  accel_steps  ramp length in steps (0 = use defaults)
 * @note   Returns immediately. Use Stepper_IsBusy() to poll completion.
 *         Motor must be idle when called.
 */
void Stepper_StartMove(uint8_t  motor, uint8_t dir, uint32_t steps,
                       uint16_t start_delay, uint16_t target_delay,
                       uint16_t accel_steps);

/* ---------- Non-Blocking Dual-Motor Overlap ---------- */

/**
 * @brief  Launch dual-motor overlapping trapezoidal move (non-blocking)
 * @param  m1           first motor ID
 * @param  steps1       total steps for motor 1
 * @param  dir1         motor 1 direction
 * @param  m2           second motor ID
 * @param  steps2       total steps for motor 2
 * @param  dir2         motor 2 direction
 * @param  m2_offset    motor 2 starts after motor 1 reaches this step count
 * @param  start_delay  µs half-cycle at ramp edges
 * @param  target_delay µs half-cycle at cruise
 * @param  accel_steps  ramp length
 * @note   Non-blocking. DIR/ENA set internally.
 *         Motor 2 starts when motor 1 reaches m2_offset.
 */
void Stepper_StartMoveOverlap(uint8_t  m1, uint32_t steps1, uint8_t dir1,
                              uint8_t  m2, uint32_t steps2, uint8_t dir2,
                              uint32_t m2_offset,
                              uint16_t start_delay, uint16_t target_delay,
                              uint16_t accel_steps);

/* ---------- Non-Blocking Dual-Motor with Direction Change ---------- */

/**
 * @brief  Launch dual-motor move where one motor changes direction mid-way
 * @param  m_cont       continuous motor ID
 * @param  steps_cont   total steps for continuous motor
 * @param  dir_cont     continuous motor direction
 * @param  m_ph         phased motor ID (changes direction mid-move)
 * @param  steps_ph1    phase-1 steps for phased motor
 * @param  dir_ph1      direction for phase 1
 * @param  steps_ph2    phase-2 steps for phased motor
 * @param  dir_ph2      direction for phase 2
 * @param  ph2_offset   phased motor phase 2 starts when continuous motor
 *                      reaches this many steps
 * @param  start_delay  µs half-cycle at ramp edges
 * @param  target_delay µs half-cycle at cruise
 * @param  accel_steps  ramp length
 * @note   Non-blocking. The phased motor runs phase1, pauses, switches
 *         direction, then runs phase2 when m_cont reaches ph2_offset.
 */
void Stepper_StartMoveOverlap2(uint8_t  m_cont,   uint32_t steps_cont,
                               uint8_t  dir_cont,
                               uint8_t  m_ph,     uint32_t steps_ph1,
                               uint8_t  dir_ph1,  uint32_t steps_ph2,
                               uint8_t  dir_ph2,  uint32_t ph2_offset,
                               uint16_t start_delay, uint16_t target_delay,
                               uint16_t accel_steps);

/**
 * @brief  Launch a cross-triggered three-segment dual-motor move
 * @note   The lead motor runs phase 1 continuously. The other motor starts
 *         at other_offset lead steps. Lead phase 2 starts at lead2_offset
 *         other-motor steps, allowing a direction reversal without a second
 *         host command.
 */
void Stepper_StartMoveOverlap3(uint8_t  m_lead,
                               uint32_t steps_lead1, uint8_t dir_lead1,
                               uint32_t steps_lead2, uint8_t dir_lead2,
                               uint8_t  m_other,
                               uint32_t steps_other, uint8_t dir_other,
                               uint32_t other_offset,
                               uint32_t lead2_offset,
                               uint16_t start_delay,
                               uint16_t target_delay,
                               uint16_t accel_steps);

/* ---------- Control ---------- */

/**
 * @brief  Emergency stop a stepper motor immediately
 * @param  motor  STEPPER_HORIZ or STEPPER_VERT
 * @note   Resets all state. Motor goes to IDLE.
 */
void Stepper_Stop(uint8_t motor);

/**
 * @brief  Check if a stepper motor is currently moving
 * @retval 1 if busy (moving), 0 if idle
 */
uint8_t Stepper_IsBusy(uint8_t motor);

/**
 * @brief  Get cumulative step position
 * @return signed step count (positive = forward, negative = reverse)
 */
int32_t Stepper_GetPosition(uint8_t motor);

/**
 * @brief  Set cumulative step position (zero reference)
 */
void Stepper_SetPosition(uint8_t motor, int32_t pos);

/* ---------- Timer Tick (called from TIM7 ISR) ---------- */

/**
 * @brief  Process stepper state machines (call from TIM7 ISR @ 100 kHz)
 * @note   Manages pulse generation and trapezoidal profiles for both motors.
 *         Must execute quickly — keep ISR lean.
 */
void Stepper_Tick(void);

/* ---------- Conversion Helper ---------- */

/**
 * @brief  Convert centimeters to steps
 */
#define CM_TO_STEPS(cm)             ((uint32_t)((cm) * STEPS_PER_CM))

/* ---------- LED Helpers ---------- */
void Stepper_LED_On(uint8_t motor);
void Stepper_LED_Off(uint8_t motor);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H__ */
