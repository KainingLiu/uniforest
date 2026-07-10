/**
 ******************************************************************************
 * @file    stepper.h
 * @brief   Stepper motor control library for DJI A-board
 *          Two TB6600-driven stepper motors, GPIO bit-bang control.
 *          Trapezoidal acceleration profiles with independent velocity curves.
 *
 *  Pin assignments (from user's A–H table):
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
 *  Mechanical: 400 steps/rev (2× microstepping), 10 mm lead → 400 steps/cm
 ******************************************************************************
 */

#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= Motor IDs ====================================== */

#define STEPPER_HORIZ               0       /* Stepper 1: horizontal (forward/back) */
#define STEPPER_VERT                1       /* Stepper 2: vertical (up/down) */
#define STEPPER_COUNT               2

/* ================= Direction & Enable (TB6600 common-cathode) ============= */

#define STEP_DIR_FORWARD            0       /* LOW → forward */
#define STEP_DIR_REVERSE            1       /* HIGH → reverse */
#define STEP_ENA_ON                 0       /* LOW → enabled (energized) */
#define STEP_ENA_OFF                1       /* HIGH → disabled (free) */

/* ======================= Speed Profile Parameters ========================= */

#define STEP_START_DELAY_US         1000    /* start speed ~500 Hz */
#define STEP_TARGET_DELAY_US        100     /* target speed ~5 kHz */
#define STEP_ACCEL_STEPS            400     /* accel / decel ramp length */

/* ======================= Steps per Centimeter ============================= */
/*  400 steps/rev  ÷  10 mm/rev  =  40 steps/mm  =  400 steps/cm              */

#define STEPS_PER_CM                400

/* ======================== delay_us Calibration ============================ */
/*  180 MHz SYSCLK ÷ 12 (loop overhead) ≈ 15  (tune with oscilloscope)       */

#define DELAY_US_COEFF              15

/* ===================== Sub-Sampling Fixed-Point =========================== */
/*  Used internally by MoveOverlap / MoveOverlap2                             */

#define SUB_SHIFT                   10
#define SUB_ONE                     (1u << SUB_SHIFT)       /* 1024 = 1.0 */

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
 * @brief  NOP-based microsecond delay
 * @param  us  delay duration in microseconds (0 → immediate return)
 * @note   Calibrated for 180 MHz SYSCLK. Coefficient: DELAY_US_COEFF (15).
 *         For delays > ~50 ms use HAL_Delay() to avoid counter overflow.
 */
void delay_us(uint32_t us);

/* ---------- Single-Motor Trapezoidal Move ---------- */

/**
 * @brief  Move one stepper with trapezoidal acceleration
 * @param  motor        STEPPER_HORIZ or STEPPER_VERT
 * @param  steps        total step count
 * @param  start_delay  initial half-cycle delay (µs) — start speed
 * @param  target_delay minimum half-cycle delay (µs) — cruise speed
 * @param  accel_steps  number of steps for accel (and decel) phase
 * @note   Blocking — returns only after all steps are completed.
 *         If steps < 2*accel_steps, the ramp is clamped proportionally.
 *         The DIR and ENA must be set before calling this function.
 */
void Stepper_MoveAccel(uint8_t  motor,
                       uint32_t steps,
                       uint16_t start_delay,
                       uint16_t target_delay,
                       uint16_t accel_steps);

/* ---------- Dual-Motor Overlapping Move ---------- */

/**
 * @brief  Move two steppers simultaneously with independent trapezoidal profiles
 * @param  m1           first motor ID
 * @param  steps1       total steps for motor 1
 * @param  m2           second motor ID
 * @param  steps2       total steps for motor 2
 * @param  m2_offset    motor 2 starts after motor 1 completes this many steps
 * @param  start_delay  µs half-cycle at start/end of ramp
 * @param  target_delay µs half-cycle at cruise speed
 * @param  accel_steps  ramp length in steps
 * @note   Blocking. DIR/ENA must be set externally before calling.
 *         Uses sub-sampling to maintain independent velocity curves.
 */
void Stepper_MoveOverlap(uint8_t  m1, uint32_t steps1,
                         uint8_t  m2, uint32_t steps2,
                         uint32_t m2_offset,
                         uint16_t start_delay,
                         uint16_t target_delay,
                         uint16_t accel_steps);

/* ---------- Dual-Motor Move with Mid-Motion Direction Change ------------- */

/**
 * @brief  Two-motor move where one motor changes direction mid-way
 * @param  m_cont       continuous motor ID
 * @param  steps_cont   total steps for continuous motor
 * @param  m_ph         phased motor ID (direction changes mid-move)
 * @param  steps_ph1    phase-1 steps for phased motor
 * @param  dir_ph1      direction for phase 1
 * @param  steps_ph2    phase-2 steps for phased motor
 * @param  dir_ph2      direction for phase 2
 * @param  ph2_offset   phased motor starts its phase 2 after m_cont
 *                      has completed this many steps
 * @param  start_delay  µs half-cycle at start/end of ramp
 * @param  target_delay µs half-cycle at cruise speed
 * @param  accel_steps  ramp length in steps
 * @note   The phased motor runs phase 1, pauses, changes direction,
 *         then runs phase 2 when m_cont reaches ph2_offset.
 */
void Stepper_MoveOverlap2(uint8_t  m_cont,   uint32_t steps_cont,
                          uint8_t  m_ph,     uint32_t steps_ph1,
                          uint8_t  dir_ph1,  uint32_t steps_ph2,
                          uint8_t  dir_ph2,  uint32_t ph2_offset,
                          uint16_t start_delay,
                          uint16_t target_delay,
                          uint16_t accel_steps);

/* ---------- Motion Utility ---------- */

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
