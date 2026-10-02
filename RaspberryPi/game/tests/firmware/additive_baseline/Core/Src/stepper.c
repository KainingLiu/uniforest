/**
 ******************************************************************************
 * @file    stepper.c
 * @brief   Non-blocking stepper motor control — TIM7 interrupt driven
 *
 *  TIM7 @ 100 kHz (10 µs tick) → Stepper_Tick() handles:
 *    - Pulse generation (PUL HIGH → delay → PUL LOW → delay)
 *    - S-curve velocity profiles (smooth accel → cruise → smooth decel)
 *    - Linked dual-motor coordination (Overlap / Overlap2 / Overlap3)
 *
 *  TB6600 drivers, common-cathode wiring.
 *  PUL: rising edge = 1 step. DIR: LOW=forward, HIGH=reverse.
 *  ENA: LOW=enabled, HIGH=disabled.
 ******************************************************************************
 */

#include "stepper.h"
#include <string.h>

/* ============================ Constants ===================================== */

#define TICK_US                     10u     /* TIM7 period: 10 µs → 100 kHz */
#define TICK_PER_MS                 100u    /* ticks per millisecond */

/* ========================== Per-Motor State ================================= */

typedef enum {
    SM_IDLE = 0,
    SM_ACCEL,
    SM_CRUISE,
    SM_DECEL,
    SM_WAIT_LINK,       /* waiting for linked motor to reach trigger step */
} StepPhase_t;

typedef struct {
    StepPhase_t phase;

    /* Current segment */
    uint8_t  dir;           /* STEP_DIR_FORWARD / STEP_DIR_REVERSE */
    uint32_t step_idx;      /* steps completed in this segment */
    uint32_t seg_steps;     /* total steps in this segment */

    /* S-curve profile (all in timer ticks, except stored originals in µs) */
    uint16_t start_delay;    /* ticks at ramp start */
    uint16_t target_delay;   /* ticks at cruise */
    uint16_t accel_steps;    /* ramp length */
    uint32_t decel_start;    /* seg_steps - accel_steps */

    /* Pulse generation */
    uint16_t tick_count;     /* ticks since last PUL toggle */
    uint16_t half_delay;     /* current half-cycle delay (ticks) */
    uint8_t  pulse_high;     /* 1 = PUL pin currently HIGH */

    /* Position tracking */
    int32_t  pos;            /* cumulative signed step count */

    /* Linked-move trigger */
    uint8_t  link_motor;     /* motor index to watch (for SM_WAIT_LINK) */
    uint32_t link_trigger;   /* step_idx threshold on link_motor */

    /* Next segment (for Overlap2 phase 2) */
    uint8_t  has_next;
    uint8_t  next_dir;
    uint32_t next_steps;
    uint32_t next_trigger;   /* trigger on the continuous motor */
    uint8_t  next_link_motor;

    /* Pins */
    const StepperPins_t *pins;
} StepperCtx_t;

/* ============================ Static State ================================== */

static StepperCtx_t g_stepper[STEPPER_COUNT];

/* TIM7 handle — public so ISR can access it */
TIM_HandleTypeDef htim7_stepper;

/* Default speed parameters (µs) */
static uint16_t g_default_start_delay  = STEP_START_DELAY_US;
static uint16_t g_default_target_delay = STEP_TARGET_DELAY_US;
static uint16_t g_default_accel_steps  = STEP_ACCEL_STEPS;

/* Pin tables */
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

/* ======================= Microsecond Delay ================================== */

void delay_us(uint32_t us)
{
    if (us == 0) return;
    uint32_t count = us * DELAY_US_COEFF;
    for (uint32_t i = 0; i < count; i++)
    {
        __NOP();
    }
}

/* ============== Internal: Configure and Start a Segment ===================== */

/**
 * @brief  Compute trapezoidal profile and set motor into ACCEL phase
 * @param  ctx          motor context (must be IDLE or WAIT_LINK)
 * @param  dir          direction
 * @param  steps        total steps for this segment
 * @param  start_d_us   µs start delay (ramp begin)
 * @param  target_d_us  µs cruise delay
 * @param  accel_s      ramp length in steps
 */
static void _stepper_launch_segment(StepperCtx_t *ctx, uint8_t motor,
                                    uint8_t dir, uint32_t steps,
                                    uint16_t start_d_us, uint16_t target_d_us,
                                    uint16_t accel_s)
{
    /* Clamp accel_steps */
    if (accel_s == 0) accel_s = 1;
    if (accel_s * 2 > steps)
    {
        accel_s = steps / 2;
        if (accel_s == 0) accel_s = 1;
    }

    /* Set direction and enable */
    Stepper_SetDir(motor, dir);
    Stepper_Enable(motor, STEP_ENA_ON);

    ctx->dir         = dir;
    ctx->step_idx    = 0;
    ctx->seg_steps   = steps;

    /* Apply the small lost-step safety margin, then convert to timer ticks. */
    uint32_t tick_den = STEP_DELAY_SCALE_DEN * TICK_US;
    uint16_t sd = (uint16_t)(((uint32_t)start_d_us * STEP_DELAY_SCALE_NUM
                            + tick_den / 2u) / tick_den);
    uint16_t td = (uint16_t)(((uint32_t)target_d_us * STEP_DELAY_SCALE_NUM
                            + tick_den / 2u) / tick_den);
    if (sd < 2) sd = 2;
    if (td < 2) td = 2;
    if (sd <= td) sd = td + 1;   /* ensure positive ramp */

    ctx->start_delay  = sd;
    ctx->target_delay = td;
    ctx->accel_steps  = accel_s;
    ctx->decel_start  = steps - accel_s;

    /* Initialize pulse state */
    ctx->half_delay  = sd;  /* start at start_delay */
    ctx->tick_count  = 0;
    ctx->pulse_high  = 0;

    ctx->phase = (accel_s > 0 && steps > 1) ? SM_ACCEL : SM_CRUISE;

    /* LED on */
    Stepper_LED_On(motor);
}

/*
 * Return the half-cycle delay for a step in the current segment.
 * The normalized smoothstep curve 3u^2 - 2u^3 has zero slope at both
 * endpoints. Q15 arithmetic keeps this ISR free of floating point.
 */
static uint16_t _stepper_scurve_delay(const StepperCtx_t *ctx,
                                     uint32_t step_idx)
{
    uint32_t u_q15;
    uint32_t curve_q15;
    uint32_t range = (uint32_t)(ctx->start_delay - ctx->target_delay);

    if (step_idx < ctx->accel_steps)
    {
        u_q15 = (step_idx * 32768u) / ctx->accel_steps;
        if (u_q15 > 32768u) u_q15 = 32768u;
        uint32_t u2_q15 = (u_q15 * u_q15) >> 15;
        uint32_t u3_q15 = (u2_q15 * u_q15) >> 15;
        curve_q15 = 3u * u2_q15 - 2u * u3_q15;
        return (uint16_t)(ctx->start_delay
               - (range * curve_q15) / 32768u);
    }

    if (step_idx >= ctx->decel_start)
    {
        uint32_t decel_i = step_idx - ctx->decel_start;
        u_q15 = (decel_i * 32768u) / ctx->accel_steps;
        if (u_q15 > 32768u) u_q15 = 32768u;
        uint32_t u2_q15 = (u_q15 * u_q15) >> 15;
        uint32_t u3_q15 = (u2_q15 * u_q15) >> 15;
        curve_q15 = 3u * u2_q15 - 2u * u3_q15;
        return (uint16_t)(ctx->target_delay
               + (range * curve_q15) / 32768u);
    }

    return ctx->target_delay;
}

/* ================== Timer Tick (called from TIM7 ISR) ======================= */

/**
 * @brief  Process both stepper state machines — called at 100 kHz
 * @note   Must be efficient. No FPU usage. No blocking calls.
 */
void Stepper_Tick(void)
{
    for (uint8_t m = 0; m < STEPPER_COUNT; m++)
    {
        StepperCtx_t *ctx = &g_stepper[m];

        /* ---- WAIT_LINK: check if trigger condition met ---- */
        if (ctx->phase == SM_WAIT_LINK)
        {
            uint8_t  lm  = ctx->link_motor;
            uint32_t trig = ctx->link_trigger;

            if (lm < STEPPER_COUNT &&
                g_stepper[lm].step_idx >= trig)
            {
                /* Trigger met — launch the waiting segment */
                _stepper_launch_segment(ctx, m, ctx->dir, ctx->seg_steps,
                                       g_default_start_delay,
                                       g_default_target_delay,
                                       g_default_accel_steps);
            }
            continue;
        }

        /* ---- IDLE: nothing to do ---- */
        if (ctx->phase == SM_IDLE)
            continue;

        /* ---- Active: process pulse generation ---- */
        ctx->tick_count++;
        if (ctx->tick_count < ctx->half_delay)
            continue;

        /* Time to toggle PUL */
        ctx->tick_count = 0;

        if (ctx->pulse_high)
        {
            /* PUL HIGH → LOW (end of half-cycle, start of second half) */
            HAL_GPIO_WritePin(ctx->pins->pul_port, ctx->pins->pul_pin, GPIO_PIN_RESET);
            ctx->pulse_high = 0;
        }
        else
        {
            /* PUL LOW → HIGH (start of new step) */
            HAL_GPIO_WritePin(ctx->pins->pul_port, ctx->pins->pul_pin, GPIO_PIN_SET);
            ctx->pulse_high = 1;

            /* A full step has completed (we were LOW, now going HIGH).
             * Note: the "full step" completes at the LOW→HIGH transition
             * because the TB6600 triggers on the rising edge. So the step
             * actually happens at this edge. */
        }

        /* Only update step counter on the HIGH edge (new step beginning) */
        if (!ctx->pulse_high)
            continue;   /* just went LOW — wait for next half-cycle */

        /* ---- Step completed: advance and recalculate delay ---- */
        ctx->step_idx++;

        /* Update position (signed) */
        if (ctx->dir == STEP_DIR_FORWARD)
            ctx->pos++;
        else
            ctx->pos--;

        /* ---- Phase transitions ---- */
        if (ctx->step_idx >= ctx->seg_steps)
        {
            /* Segment complete */
            Stepper_LED_Off(m);

            if (ctx->has_next)
            {
                /* Load next segment */
                ctx->dir         = ctx->next_dir;
                ctx->seg_steps   = ctx->next_steps;
                ctx->link_motor  = ctx->next_link_motor;
                ctx->link_trigger= ctx->next_trigger;
                ctx->has_next    = 0;

                /* Check if trigger already met */
                uint8_t lm2 = ctx->link_motor;
                if (lm2 < STEPPER_COUNT &&
                    g_stepper[lm2].step_idx >= ctx->link_trigger)
                {
                    _stepper_launch_segment(ctx, m, ctx->dir, ctx->seg_steps,
                                           g_default_start_delay,
                                           g_default_target_delay,
                                           g_default_accel_steps);
                }
                else
                {
                    /* Wait for trigger */
                    ctx->phase     = SM_WAIT_LINK;
                    ctx->step_idx  = 0;
                    ctx->pulse_high = 0;
                    ctx->tick_count = 0;
                }
            }
            else
            {
                /* Fully done */
                ctx->phase     = SM_IDLE;
                ctx->pulse_high = 0;
                ctx->tick_count = 0;
                /* Keep ENA on to hold position */
            }
            continue;
        }

        /* ---- Recalculate half_delay for next step using S-curve ---- */
        ctx->half_delay = _stepper_scurve_delay(ctx, ctx->step_idx);
        if (ctx->step_idx < ctx->accel_steps)
            ctx->phase = SM_ACCEL;
        else if (ctx->step_idx >= ctx->decel_start)
            ctx->phase = SM_DECEL;
        else
            ctx->phase = SM_CRUISE;
    }
}

/* ========================== Initialization ================================== */

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

    /* Set initial states */
    for (int i = 0; i < STEPPER_COUNT; i++)
    {
        HAL_GPIO_WritePin(stepper_pins[i].pul_port,
                          stepper_pins[i].pul_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(stepper_pins[i].dir_port,
                          stepper_pins[i].dir_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(stepper_pins[i].ena_port,
                          stepper_pins[i].ena_pin, GPIO_PIN_RESET);

        /* Init context */
        memset(&g_stepper[i], 0, sizeof(StepperCtx_t));
        g_stepper[i].pins  = &stepper_pins[i];
        g_stepper[i].phase = SM_IDLE;
    }


    /* ---- TIM7: 100 kHz tick for stepper pulse generation ---- */
    /* APB1 timer clock = 90 MHz → 900 counts = 100 kHz (10 µs) */
    __HAL_RCC_TIM7_CLK_ENABLE();

    memset(&htim7_stepper, 0, sizeof(htim7_stepper));
    htim7_stepper.Instance               = TIM7;
    htim7_stepper.Init.Prescaler         = 0;
    htim7_stepper.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim7_stepper.Init.Period            = 899;   /* 90 MHz / 900 = 100 kHz */
    htim7_stepper.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim7_stepper.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim7_stepper);

    /* Enable update interrupt and start timer */
    __HAL_TIM_ENABLE_IT(&htim7_stepper, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim7_stepper);

    /* NVIC: TIM7 IRQ, priority 1 (higher than UARTs at 2-3) */
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* ======================== Low-Level Control ================================= */

void Stepper_SetDir(uint8_t motor, uint8_t dir)
{
    if (motor >= STEPPER_COUNT) return;
    HAL_GPIO_WritePin(stepper_pins[motor].dir_port, stepper_pins[motor].dir_pin,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Enable(uint8_t motor, uint8_t ena)
{
    if (motor >= STEPPER_COUNT) return;
    HAL_GPIO_WritePin(stepper_pins[motor].ena_port, stepper_pins[motor].ena_pin,
                      ena ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ======================== Parameter Configuration =========================== */

void Stepper_SetParams(uint16_t start_delay, uint16_t target_delay,
                       uint16_t accel_steps)
{
    g_default_start_delay  = start_delay;
    g_default_target_delay = target_delay;
    g_default_accel_steps  = accel_steps;
}

/* ===================== Non-Blocking Move API ================================ */

/**
 * @brief  Launch single-motor trapezoidal move (non-blocking)
 */
void Stepper_StartMove(uint8_t motor, uint8_t dir, uint32_t steps,
                       uint16_t start_delay, uint16_t target_delay,
                       uint16_t accel_steps)
{
    if (motor >= STEPPER_COUNT || steps == 0) return;
    StepperCtx_t *ctx = &g_stepper[motor];

    /* Use defaults if parameters are 0 */
    if (start_delay  == 0) start_delay  = g_default_start_delay;
    if (target_delay == 0) target_delay = g_default_target_delay;
    if (accel_steps  == 0) accel_steps  = g_default_accel_steps;

    /* Reset linked-move state */
    ctx->has_next   = 0;
    ctx->link_motor = 0;
    ctx->link_trigger = 0;

    _stepper_launch_segment(ctx, motor, dir, steps,
                            start_delay, target_delay, accel_steps);
}

/**
 * @brief  Launch dual-motor overlapping move (non-blocking)
 */
void Stepper_StartMoveOverlap(uint8_t  m1, uint32_t steps1, uint8_t dir1,
                              uint8_t  m2, uint32_t steps2, uint8_t dir2,
                              uint32_t m2_offset,
                              uint16_t start_delay, uint16_t target_delay,
                              uint16_t accel_steps)
{
    if (m1 >= STEPPER_COUNT || m2 >= STEPPER_COUNT) return;
    if (m1 == m2) return;

    if (start_delay  == 0) start_delay  = g_default_start_delay;
    if (target_delay == 0) target_delay = g_default_target_delay;
    if (accel_steps  == 0) accel_steps  = g_default_accel_steps;

    /* Motor 1: start immediately */
    Stepper_StartMove(m1, dir1, steps1, start_delay, target_delay, accel_steps);

    /* Motor 2: wait until motor 1 reaches m2_offset */
    StepperCtx_t *ctx2 = &g_stepper[m2];
    memset(ctx2, 0, sizeof(StepperCtx_t));
    ctx2->pins        = &stepper_pins[m2];
    ctx2->phase       = SM_WAIT_LINK;
    ctx2->dir         = dir2;
    ctx2->seg_steps   = steps2;
    ctx2->link_motor  = m1;
    ctx2->link_trigger= m2_offset;
    ctx2->has_next    = 0;

    /* Store speed params for when the wait ends */
    g_default_start_delay  = start_delay;
    g_default_target_delay = target_delay;
    g_default_accel_steps  = accel_steps;
}

/**
 * @brief  Launch dual-motor overlapping move with mid-move direction change
 */
void Stepper_StartMoveOverlap2(uint8_t  m_cont,   uint32_t steps_cont,
                               uint8_t  dir_cont,
                               uint8_t  m_ph,     uint32_t steps_ph1,
                               uint8_t  dir_ph1,  uint32_t steps_ph2,
                               uint8_t  dir_ph2,  uint32_t ph2_offset,
                               uint16_t start_delay, uint16_t target_delay,
                               uint16_t accel_steps)
{
    if (m_cont >= STEPPER_COUNT || m_ph >= STEPPER_COUNT) return;
    if (m_cont == m_ph) return;

    if (start_delay  == 0) start_delay  = g_default_start_delay;
    if (target_delay == 0) target_delay = g_default_target_delay;
    if (accel_steps  == 0) accel_steps  = g_default_accel_steps;

    /* Save params for use by the phased motor's segments */
    g_default_start_delay  = start_delay;
    g_default_target_delay = target_delay;
    g_default_accel_steps  = accel_steps;

    /* Continuous motor: start immediately */
    Stepper_StartMove(m_cont, dir_cont, steps_cont,
                      start_delay, target_delay, accel_steps);

    /* Phased motor: phase 1 starts immediately (offset = 0) */
    StepperCtx_t *ctx_ph = &g_stepper[m_ph];
    memset(ctx_ph, 0, sizeof(StepperCtx_t));
    ctx_ph->pins = &stepper_pins[m_ph];

    if (steps_ph1 > 0)
    {
        /* Phase 1: start immediately */
        _stepper_launch_segment(ctx_ph, m_ph, dir_ph1, steps_ph1,
                                start_delay, target_delay, accel_steps);

        /* Queue phase 2 */
        ctx_ph->has_next        = 1;
        ctx_ph->next_dir        = dir_ph2;
        ctx_ph->next_steps      = steps_ph2;
        ctx_ph->next_trigger    = ph2_offset;
        ctx_ph->next_link_motor = m_cont;
    }
    else if (steps_ph2 > 0)
    {
        /* No phase 1 — wait for phase 2 offset directly */
        ctx_ph->phase        = SM_WAIT_LINK;
        ctx_ph->dir          = dir_ph2;
        ctx_ph->seg_steps    = steps_ph2;
        ctx_ph->link_motor   = m_cont;
        ctx_ph->link_trigger = ph2_offset;
        ctx_ph->has_next     = 0;
    }
}

/**
 * @brief  Launch a cross-triggered three-segment dual-motor move
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
                               uint16_t accel_steps)
{
    if (m_lead >= STEPPER_COUNT || m_other >= STEPPER_COUNT) return;
    if (m_lead == m_other) return;
    if (steps_lead1 == 0 || steps_lead2 == 0 || steps_other == 0) return;
    if (other_offset > steps_lead1 || lead2_offset > steps_other) return;

    if (start_delay  == 0) start_delay  = g_default_start_delay;
    if (target_delay == 0) target_delay = g_default_target_delay;
    if (accel_steps  == 0) accel_steps  = g_default_accel_steps;

    g_default_start_delay  = start_delay;
    g_default_target_delay = target_delay;
    g_default_accel_steps  = accel_steps;

    /* Lead phase 1 is one uninterrupted segment. Queue phase 2 so it starts
     * when the other motor reaches its trigger distance. */
    Stepper_StartMove(m_lead, dir_lead1, steps_lead1,
                      start_delay, target_delay, accel_steps);
    StepperCtx_t *ctx_lead = &g_stepper[m_lead];
    ctx_lead->has_next        = 1;
    ctx_lead->next_dir        = dir_lead2;
    ctx_lead->next_steps      = steps_lead2;
    ctx_lead->next_trigger    = lead2_offset;
    ctx_lead->next_link_motor = m_other;

    /* The other motor waits for the lead motor's first trigger. Preserve its
     * cumulative position while resetting only the active move state. */
    StepperCtx_t *ctx_other = &g_stepper[m_other];
    int32_t other_pos = ctx_other->pos;
    memset(ctx_other, 0, sizeof(StepperCtx_t));
    ctx_other->pos          = other_pos;
    ctx_other->pins         = &stepper_pins[m_other];
    ctx_other->phase        = SM_WAIT_LINK;
    ctx_other->dir          = dir_other;
    ctx_other->seg_steps    = steps_other;
    ctx_other->link_motor   = m_lead;
    ctx_other->link_trigger = other_offset;
    ctx_other->has_next     = 0;
}

/* ======================== Control ========================================== */

void Stepper_Stop(uint8_t motor)
{
    if (motor >= STEPPER_COUNT) return;
    StepperCtx_t *ctx = &g_stepper[motor];

    /* Reset pulse pin to LOW */
    HAL_GPIO_WritePin(ctx->pins->pul_port, ctx->pins->pul_pin, GPIO_PIN_RESET);

    /* Clear all state */
    memset(ctx, 0, sizeof(StepperCtx_t));
    ctx->pins  = &stepper_pins[motor];
    ctx->phase = SM_IDLE;

    Stepper_LED_Off(motor);
}

uint8_t Stepper_IsBusy(uint8_t motor)
{
    if (motor >= STEPPER_COUNT) return 0;
    return (g_stepper[motor].phase != SM_IDLE) ? 1 : 0;
}

int32_t Stepper_GetPosition(uint8_t motor)
{
    if (motor >= STEPPER_COUNT) return 0;
    return g_stepper[motor].pos;
}

void Stepper_SetPosition(uint8_t motor, int32_t pos)
{
    if (motor >= STEPPER_COUNT) return;
    g_stepper[motor].pos = pos;
}

/* ========================= LED Helpers ====================================== */

void Stepper_LED_On(uint8_t motor)
{
    /* PD12/PD13 are now reserved for the suction pump and valve. */
    (void)motor;
}

void Stepper_LED_Off(uint8_t motor)
{
    /* PD12/PD13 are now reserved for the suction pump and valve. */
    (void)motor;
}
