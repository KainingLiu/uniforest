/**
 ******************************************************************************
 * @file    stepper.c
 * @brief   Stepper motor control implementation
 *          TB6600 drivers, common-cathode wiring, GPIO bit-bang.
 *
 *  PUL: rising edge = 1 step (HIGH → optocoupler ON)
 *  DIR: LOW = forward, HIGH = reverse
 *  ENA: LOW = enabled (energized), HIGH = disabled (free)
 ******************************************************************************
 */

#include "stepper.h"

/* ============================ Private Variables =========================== */

static const StepperPins_t stepper_pins[STEPPER_COUNT] = {
    [STEPPER_HORIZ] = {
        STEP1_PUL_PORT, STEP1_PUL_PIN,
        STEP1_DIR_PORT, STEP1_DIR_PIN,
        STEP1_ENA_PORT, STEP1_ENA_PIN
    },
    [STEPPER_VERT] = {
        STEP2_PUL_PORT, STEP2_PUL_PIN,
        STEP2_DIR_PORT, STEP2_DIR_PIN,
        STEP2_ENA_PORT, STEP2_ENA_PIN
    },
};

/* ======================= Microsecond Delay ================================ */

/**
 * @brief  NOP-based microsecond delay
 * @param  us  delay in µs (0 → immediate return)
 * @note   Calibrated for 180 MHz SYSCLK.
 *         Coefficient DELAY_US_COEFF = 15.
 *         A guard prevents infinite loops at us = 0.
 */
void delay_us(uint32_t us)
{
    if (us == 0) return;
    uint32_t count = us * DELAY_US_COEFF;
    for (uint32_t i = 0; i < count; i++)
    {
        __NOP();
    }
}

/* ========================== Initialization ================================ */

/**
 * @brief  Initialize stepper GPIOs and LED pins
 * @note   Configures all PUL/DIR/ENA pins as output push-pull.
 *         PUL pins: high speed. DIR/ENA pins: low speed.
 *         LED pins: output push-pull, initially OFF.
 *         Sets initial state: PUL=LOW, DIR=FORWARD, ENA=ON.
 */
void Stepper_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;

    /* --- Stepper1 (Horizontal) --- */
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* PI0 = PUL (high speed) */
    gpio.Pin   = STEP1_PUL_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STEP1_PUL_PORT, &gpio);

    /* PH12 = DIR, PH11 = ENA (low speed) */
    gpio.Pin   = STEP1_DIR_PIN | STEP1_ENA_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STEP1_DIR_PORT, &gpio);

    /* --- Stepper2 (Vertical) --- */
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PH10 = PUL (high speed) */
    gpio.Pin   = STEP2_PUL_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STEP2_PUL_PORT, &gpio);

    /* PD15 = DIR, PD14 = ENA (low speed) */
    gpio.Pin   = STEP2_DIR_PIN | STEP2_ENA_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STEP2_DIR_PORT, &gpio);

    /* --- LED pins (PD13, PD12) --- */
    gpio.Pin   = LED_STEPPER1_PIN | LED_STEPPER2_PIN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_STEPPER1_PORT, &gpio);

    /* Set initial states */
    for (int i = 0; i < STEPPER_COUNT; i++)
    {
        HAL_GPIO_WritePin(stepper_pins[i].pul_port, stepper_pins[i].pul_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(stepper_pins[i].dir_port, stepper_pins[i].dir_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(stepper_pins[i].ena_port, stepper_pins[i].ena_pin, GPIO_PIN_RESET);
    }

    /* LEDs off */
    Stepper_LED_Off(STEPPER_HORIZ);
    Stepper_LED_Off(STEPPER_VERT);
}

/* ======================== Low-Level Control =============================== */

void Stepper_SetDir(uint8_t motor, uint8_t dir)
{
    if (motor >= STEPPER_COUNT) return;
    HAL_GPIO_WritePin(stepper_pins[motor].dir_port, stepper_pins[motor].dir_pin,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Enable(uint8_t motor, uint8_t ena)
{
    if (motor >= STEPPER_COUNT) return;
    /* ENA = LOW → enabled, HIGH → disabled */
    HAL_GPIO_WritePin(stepper_pins[motor].ena_port, stepper_pins[motor].ena_pin,
                      ena ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ============================ Pulse Generation ============================ */

/**
 * @brief  Generate a single stepper pulse
 * @param  motor        STEPPER_HORIZ or STEPPER_VERT
 * @param  delay_us_val half-cycle delay in µs
 * @note   PUL HIGH → delay → PUL LOW → delay
 */
static void Stepper_Pulse(uint8_t motor, uint16_t delay_us_val)
{
    const StepperPins_t *p = &stepper_pins[motor];

    HAL_GPIO_WritePin(p->pul_port, p->pul_pin, GPIO_PIN_SET);
    delay_us(delay_us_val);
    HAL_GPIO_WritePin(p->pul_port, p->pul_pin, GPIO_PIN_RESET);
    delay_us(delay_us_val);
}

/* ==================== Trapezoidal Acceleration ============================ */

/**
 * @brief  Single motor move with trapezoidal velocity profile
 *
 *  Speed profile:
 *    ┌──────────────┐
 *    │  accel  │ cruise │ decel  │
 *    │ ─────── │ ────── │ ───────│
 *    │  ramp   │ steady │  ramp  │
 *    └──────────────┘
 */
void Stepper_MoveAccel(uint8_t motor, uint32_t steps,
                       uint16_t start_delay, uint16_t target_delay,
                       uint16_t accel_steps)
{
    if (motor >= STEPPER_COUNT || steps == 0) return;

    /* Clamp accel_steps if total distance is too short for full ramp */
    if (accel_steps * 2 > steps)
    {
        accel_steps = steps / 2;
    }

    uint32_t decel_start = steps - accel_steps;  /* first step of deceleration */
    uint16_t current_delay = start_delay;

    /* Pre-calculate delay decrement per step (×100 fixed-point precision) */
    uint32_t delta_scaled = ((uint32_t)(start_delay - target_delay) * 100) / accel_steps;

    for (uint32_t i = 0; i < steps; i++)
    {
        if (i < accel_steps)
        {
            /* Acceleration: decreasing delay */
            current_delay = start_delay - (uint16_t)((delta_scaled * i) / 100);
        }
        else if (i >= decel_start)
        {
            /* Deceleration: increasing delay */
            uint32_t decel_i = i - decel_start;
            current_delay = target_delay + (uint16_t)((delta_scaled * decel_i) / 100);
        }
        else
        {
            /* Constant speed (cruise) */
            current_delay = target_delay;
        }

        Stepper_Pulse(motor, current_delay);
    }
}

/* =================== Dual-Motor Overlapping Move ========================== */

/**
 * @brief  Two steppers simultaneously, each with independent trapezoidal profile
 *
 *  Algorithm: runs a "fast clock" at the shorter of the two motor delays.
 *  The slower motor is sub-sampled via a fixed-point accumulator.
 *  Neither motor is ever forced off its own velocity curve.
 */
void Stepper_MoveOverlap(uint8_t  m1, uint32_t steps1,
                         uint8_t  m2, uint32_t steps2,
                         uint32_t m2_offset,
                         uint16_t start_delay, uint16_t target_delay,
                         uint16_t accel_steps)
{
    if (m1 >= STEPPER_COUNT || m2 >= STEPPER_COUNT) return;

    /* ---- Motor 1 independent profile ---- */
    uint32_t a1 = accel_steps;
    if (a1 * 2 > steps1) a1 = steps1 / 2;
    if (a1 == 0) a1 = 1;
    uint32_t m1_decel = steps1 - a1;
    uint32_t d1_scaled = ((uint32_t)(start_delay - target_delay) * 100) / a1;

    /* ---- Motor 2 independent profile ---- */
    uint32_t a2 = accel_steps;
    if (a2 * 2 > steps2) a2 = steps2 / 2;
    if (a2 == 0) a2 = 1;
    uint32_t m2_decel = steps2 - a2;
    uint32_t d2_scaled = ((uint32_t)(start_delay - target_delay) * 100) / a2;

    /* Per-motor step counters */
    uint32_t s1 = 0, s2 = 0;

    /* Subsampling accumulators (fixed-point, SUB_ONE = one full step) */
    uint32_t acc1 = 0, acc2 = 0;

    const StepperPins_t *p1 = &stepper_pins[m1];
    const StepperPins_t *p2 = &stepper_pins[m2];

    /* Helper: compute current half-cycle delay from motor's own profile */
    #define MOTOR_DELAY(d_scaled, s, a, decel) \
        ((s) < (a) \
            ? start_delay - (uint16_t)(((d_scaled) * (s)) / 100) \
            : (s) >= (decel) \
                ? target_delay + (uint16_t)(((d_scaled) * ((s) - (decel))) / 100) \
                : target_delay)

    while (s1 < steps1 || s2 < steps2)
    {
        uint8_t m1_on = (s1 < steps1);
        uint8_t m2_on = (s2 < steps2) && (s1 >= m2_offset);

        /* Each motor's current half-cycle delay */
        uint16_t d1 = m1_on ? MOTOR_DELAY(d1_scaled, s1, a1, m1_decel) : 0;
        uint16_t d2 = m2_on ? MOTOR_DELAY(d2_scaled, s2, a2, m2_decel) : 0;

        /* Fast clock = shorter delay among active motors */
        uint16_t d_fast;
        if      ( m1_on && !m2_on) d_fast = d1;
        else if (!m1_on &&  m2_on) d_fast = d2;
        else                       d_fast = (d1 < d2) ? d1 : d2;

        /* Subsampling ratio: fraction of a step per fast-clock cycle */
        uint32_t r1 = m1_on ? ((uint32_t)d_fast << SUB_SHIFT) / d1 : 0;
        uint32_t r2 = m2_on ? ((uint32_t)d_fast << SUB_SHIFT) / d2 : 0;

        /* Accumulate */
        acc1 += r1;
        acc2 += r2;

        uint8_t fire1 = (acc1 >= SUB_ONE);
        uint8_t fire2 = (acc2 >= SUB_ONE);
        if (fire1) { acc1 -= SUB_ONE; }
        if (fire2) { acc2 -= SUB_ONE; }

        /* Pulse the motor(s) whose step is due; wait one fast-clock period */
        if (fire1) HAL_GPIO_WritePin(p1->pul_port, p1->pul_pin, GPIO_PIN_SET);
        if (fire2) HAL_GPIO_WritePin(p2->pul_port, p2->pul_pin, GPIO_PIN_SET);
        delay_us(d_fast);
        if (fire1) HAL_GPIO_WritePin(p1->pul_port, p1->pul_pin, GPIO_PIN_RESET);
        if (fire2) HAL_GPIO_WritePin(p2->pul_port, p2->pul_pin, GPIO_PIN_RESET);
        delay_us(d_fast);

        if (fire1) s1++;
        if (fire2) s2++;
    }
    #undef MOTOR_DELAY
}

/* ============= Dual-Motor Overlap with Mid-Motion Direction Change ======== */

/**
 * @brief  Two-motor move where m_ph changes direction mid-way
 *
 *  Phase 1: m_ph runs dir_ph1 for steps_ph1, overlapping m_cont from start.
 *  Idle:     m_ph pauses while m_cont continues.
 *  Phase 2:  when m_cont reaches ph2_offset, m_ph switches to dir_ph2
 *            and runs steps_ph2, overlapping m_cont's tail.
 */
void Stepper_MoveOverlap2(
        uint8_t  m_cont,   uint32_t steps_cont,
        uint8_t  m_ph,     uint32_t steps_ph1, uint8_t dir_ph1,
                           uint32_t steps_ph2, uint8_t dir_ph2,
        uint32_t ph2_offset,
        uint16_t start_delay, uint16_t target_delay,
        uint16_t accel_steps)
{
    if (m_cont >= STEPPER_COUNT || m_ph >= STEPPER_COUNT) return;

    /* ---- Continuous motor profile ---- */
    uint32_t ac = accel_steps;
    if (ac * 2 > steps_cont) ac = steps_cont / 2;
    if (ac == 0) ac = 1;
    uint32_t cont_decel = steps_cont - ac;
    uint32_t dc_scaled = ((uint32_t)(start_delay - target_delay) * 100) / ac;

    /* ---- Phased motor: independent profile for each phase ---- */
    uint32_t ap1 = accel_steps;
    if (ap1 * 2 > steps_ph1) ap1 = steps_ph1 / 2;
    if (ap1 == 0 && steps_ph1 > 0) ap1 = 1;
    uint32_t ph1_decel = steps_ph1 - ap1;
    uint32_t dp1_scaled = steps_ph1
        ? ((uint32_t)(start_delay - target_delay) * 100) / (ap1 ? ap1 : 1) : 0;

    uint32_t ap2 = accel_steps;
    if (ap2 * 2 > steps_ph2) ap2 = steps_ph2 / 2;
    if (ap2 == 0 && steps_ph2 > 0) ap2 = 1;
    uint32_t ph2_decel = steps_ph2 - ap2;
    uint32_t dp2_scaled = steps_ph2
        ? ((uint32_t)(start_delay - target_delay) * 100) / (ap2 ? ap2 : 1) : 0;

    /* Step counters */
    uint32_t s_cont = 0;
    uint32_t s_ph   = 0;

    /* Phase state: 0=phase1, 1=idle, 2=phase2, 3=done */
    uint8_t ph_state = (steps_ph1 > 0) ? 0 : 1;

    /* Subsampling accumulators */
    uint32_t acc_cont = 0, acc_ph = 0;

    const StepperPins_t *pc = &stepper_pins[m_cont];
    const StepperPins_t *pp = &stepper_pins[m_ph];

    /* Set initial direction for phased motor */
    Stepper_SetDir(m_ph, dir_ph1);

    #define MOTOR_DELAY(d_scaled, s, a, decel) \
        ((s) < (a) \
            ? start_delay - (uint16_t)(((d_scaled) * (s)) / 100) \
            : (s) >= (decel) \
                ? target_delay + (uint16_t)(((d_scaled) * ((s) - (decel))) / 100) \
                : target_delay)

    while (s_cont < steps_cont || ph_state < 3)
    {
        /* ---- State transitions ---- */
        if (ph_state == 0 && s_ph >= steps_ph1)
        {
            ph_state = 1;
            acc_ph = 0;
        }
        if (ph_state == 1 && s_cont >= ph2_offset && steps_ph2 > 0)
        {
            ph_state = 2;
            s_ph = 0;
            acc_ph = 0;
            Stepper_SetDir(m_ph, dir_ph2);
        }
        if (ph_state == 2 && s_ph >= steps_ph2)
        {
            ph_state = 3;
        }

        uint8_t cont_on = (s_cont < steps_cont);
        uint8_t ph_on   = (ph_state == 0 || ph_state == 2);

        /* Current delays */
        uint16_t d_cont = cont_on ? MOTOR_DELAY(dc_scaled, s_cont, ac, cont_decel) : 0;
        uint16_t d_ph;
        if (!ph_on)
            d_ph = 0;
        else if (ph_state == 0)
            d_ph = MOTOR_DELAY(dp1_scaled, s_ph, ap1, ph1_decel);
        else /* ph_state == 2 */
            d_ph = MOTOR_DELAY(dp2_scaled, s_ph, ap2, ph2_decel);

        /* Fast clock */
        uint16_t d_fast;
        if      ( cont_on && !ph_on) d_fast = d_cont;
        else if (!cont_on &&  ph_on) d_fast = d_ph;
        else                          d_fast = (d_cont < d_ph) ? d_cont : d_ph;

        /* Subsampling ratios */
        uint32_t r_cont = cont_on ? ((uint32_t)d_fast << SUB_SHIFT) / d_cont : 0;
        uint32_t r_ph   = ph_on   ? ((uint32_t)d_fast << SUB_SHIFT) / d_ph   : 0;

        acc_cont += r_cont;
        acc_ph   += r_ph;

        uint8_t fire_cont = (acc_cont >= SUB_ONE);
        uint8_t fire_ph   = (acc_ph   >= SUB_ONE);
        if (fire_cont) { acc_cont -= SUB_ONE; }
        if (fire_ph)   { acc_ph   -= SUB_ONE; }

        /* Pulse */
        if (fire_cont) HAL_GPIO_WritePin(pc->pul_port, pc->pul_pin, GPIO_PIN_SET);
        if (fire_ph)   HAL_GPIO_WritePin(pp->pul_port, pp->pul_pin, GPIO_PIN_SET);
        delay_us(d_fast);
        if (fire_cont) HAL_GPIO_WritePin(pc->pul_port, pc->pul_pin, GPIO_PIN_RESET);
        if (fire_ph)   HAL_GPIO_WritePin(pp->pul_port, pp->pul_pin, GPIO_PIN_RESET);
        delay_us(d_fast);

        if (fire_cont) s_cont++;
        if (fire_ph)   s_ph++;
    }
    #undef MOTOR_DELAY
}

/* ========================= LED Helpers ==================================== */

void Stepper_LED_On(uint8_t motor)
{
    switch (motor)
    {
        case STEPPER_HORIZ:
            HAL_GPIO_WritePin(LED_STEPPER1_PORT, LED_STEPPER1_PIN, GPIO_PIN_SET);
            break;
        case STEPPER_VERT:
            HAL_GPIO_WritePin(LED_STEPPER2_PORT, LED_STEPPER2_PIN, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

void Stepper_LED_Off(uint8_t motor)
{
    switch (motor)
    {
        case STEPPER_HORIZ:
            HAL_GPIO_WritePin(LED_STEPPER1_PORT, LED_STEPPER1_PIN, GPIO_PIN_RESET);
            break;
        case STEPPER_VERT:
            HAL_GPIO_WritePin(LED_STEPPER2_PORT, LED_STEPPER2_PIN, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}
