/**
 ******************************************************************************
 * @file    actions.c
 * @brief   Action sequences: Build() and Grap()
 *          Ported from uniforest_0616, adapted to new pin assignments.
 *
 *  Servo mapping (ref channel → new ID):
 *    Ref ch1 → SERVO_GRIPPER   (0)  — gripper
 *    Ref ch2 → SERVO_ARM_FRONT (1)  — arm front
 *    Ref ch3 → SERVO_HATCH_A   (2)  — hatch A
 *    Ref ch4 → SERVO_HATCH_B   (3)  — hatch B
 *
 *  Stepper mapping (ref index → new ID):
 *    Ref STEPPER1 → STEPPER_HORIZ  — horizontal (forward/back)
 *    Ref STEPPER2 → STEPPER_VERT   — vertical (up/down)
 ******************************************************************************
 */

#include "actions.h"
#include "servo.h"
#include "stepper.h"
#include "remote_control.h"

/* ======================= Convenience Helpers ============================== */

void Actions_ServoHome(void)
{
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);   /* 84° */
    Servo_SetAngle(SERVO_ARM_FRONT, ANGLE_ARM_FRONT_DOWN); /* 90° */
    Servo_SetAngle(SERVO_HATCH_A,   ANGLE_HATCH_A_CLOSED); /* 63° */
    Servo_SetAngle(SERVO_HATCH_B,   ANGLE_HATCH_B_CLOSED); /* 117° */
    HAL_Delay(500);
}

void Actions_HatchOpen(void)
{
    Servo_SetAngle(SERVO_HATCH_A, ANGLE_HATCH_A_OPEN);    /* 130° */
    Servo_SetAngle(SERVO_HATCH_B, ANGLE_HATCH_B_OPEN);    /* 50° */
    HAL_Delay(500);
}

void Actions_HatchClose(void)
{
    Servo_SetAngle(SERVO_HATCH_A, ANGLE_HATCH_A_CLOSED);  /* 63° */
    Servo_SetAngle(SERVO_HATCH_B, ANGLE_HATCH_B_CLOSED);  /* 117° */
    HAL_Delay(500);
}

/* ========================== Build() ======================================= */

/**
 * @brief  24-step pick-and-place merged sequence
 * @note   Horz displacement balance: +5+18-23+23-23+23-23 = 0  ✓
 *         Vert displacement balance: -1-18+10-2-3+5-12+21-4+4 = 0  ✓
 */
void Actions_Build(void)
{
    /* ---- 1. Servo init (home all) ---- */
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    Servo_SetAngle(SERVO_HATCH_A,   63);
    Servo_SetAngle(SERVO_HATCH_B,   117);
    HAL_Delay(500);

    /* ---- 2. Hatch open ---- */
    Servo_SetAngle(SERVO_HATCH_A, 130);
    Servo_SetAngle(SERVO_HATCH_B, 50);
    HAL_Delay(500);

    /* ---- 3. fwd 5cm ‖ down 1cm (offset=0) ---- */
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_HORIZ,  5 * STEPS_PER_CM,
                        STEPPER_VERT,   1 * STEPS_PER_CM, 0,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 4. Gripper close ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_CLOSE);
    HAL_Delay(500);

    /* ---- 5. Arm front lift (smooth decel to 0° limit) ---- */
    Servo_SetAngle(SERVO_ARM_FRONT, 50);
    HAL_Delay(80);
    Servo_SetAngle(SERVO_ARM_FRONT, 28);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 14);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 6);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 2);
    HAL_Delay(100);
    Servo_SetAngle(SERVO_ARM_FRONT, 0);
    HAL_Delay(50);

    /* ---- 6. fwd 18cm → after 3cm: down 18cm ---- */
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_HORIZ, 18 * STEPS_PER_CM,
                        STEPPER_VERT,  18 * STEPS_PER_CM, 3 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 7. Gripper open ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_OPEN);
    HAL_Delay(500);

    /* ---- 8a. up 10cm → after 3cm: back 23cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_VERT,  10 * STEPS_PER_CM,
                        STEPPER_HORIZ, 23 * STEPS_PER_CM, 3 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 8b. down 2cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveAccel(STEPPER_VERT, 2 * STEPS_PER_CM,
                      STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 9. Arm front lower ---- */
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    HAL_Delay(500);

    /* ---- 10. Gripper close ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_CLOSE);
    HAL_Delay(500);

    /* ---- 11. Arm front lift (smooth decel to 0° limit) ---- */
    Servo_SetAngle(SERVO_ARM_FRONT, 50);
    HAL_Delay(80);
    Servo_SetAngle(SERVO_ARM_FRONT, 28);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 14);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 6);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 2);
    HAL_Delay(100);
    Servo_SetAngle(SERVO_ARM_FRONT, 0);
    HAL_Delay(50);

    /* ---- 12. up2 ‖ fwd23 → after fwd21: down5 (MoveOverlap2) ---- */
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveOverlap2(
        STEPPER_HORIZ, 23 * STEPS_PER_CM,
        STEPPER_VERT,
         2 * STEPS_PER_CM, STEP_DIR_FORWARD,
         5 * STEPS_PER_CM, STEP_DIR_REVERSE,
        21 * STEPS_PER_CM,
        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 13. Gripper open ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_OPEN);
    HAL_Delay(500);

    /* ---- 14. up 5cm → after 1cm: back 23cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_VERT,   5 * STEPS_PER_CM,
                        STEPPER_HORIZ, 23 * STEPS_PER_CM, 1 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 15. Arm front lower ---- */
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    HAL_Delay(500);

    /* ---- 16. down 12cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveAccel(STEPPER_VERT, 12 * STEPS_PER_CM,
                      STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 17. Gripper close ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_CLOSE);
    HAL_Delay(500);

    /* ---- 18. Rise 21cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveAccel(STEPPER_VERT, 21 * STEPS_PER_CM,
                      STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 19. Arm front lift (smooth decel to 0° limit) ---- */
    Servo_SetAngle(SERVO_ARM_FRONT, 50);
    HAL_Delay(80);
    Servo_SetAngle(SERVO_ARM_FRONT, 28);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 14);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 6);
    HAL_Delay(90);
    Servo_SetAngle(SERVO_ARM_FRONT, 2);
    HAL_Delay(100);
    Servo_SetAngle(SERVO_ARM_FRONT, 0);
    HAL_Delay(50);

    /* ---- 20. fwd 23cm → after 22cm: down 4cm ---- */
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_HORIZ, 23 * STEPS_PER_CM,
                        STEPPER_VERT,   4 * STEPS_PER_CM, 22 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 21. Gripper open ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_OPEN);
    HAL_Delay(500);

    /* ---- 22. up 4cm ‖ back 23cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_VERT,   4 * STEPS_PER_CM,
                        STEPPER_HORIZ, 23 * STEPS_PER_CM, 0,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 23. Arm front lower ---- */
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    HAL_Delay(500);

    /* ---- 24. Final servo init (hatch close) ---- */
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    Servo_SetAngle(SERVO_HATCH_A,   63);
    Servo_SetAngle(SERVO_HATCH_B,   117);
    HAL_Delay(500);

    /* ---- 25. Wait CH6 return to centre (1400–1600us) to prevent re-trigger ---- */
    while (1)
    {
        if (SBUS_IsValid())
        {
            uint16_t ch6 = SBUS_GetChannel(5);
            if (ch6 > SBUS_CH6_ARM_LOW && ch6 < SBUS_CH6_ARM_HIGH)
                break;
        }
        HAL_Delay(10);
    }
}

/* ========================== Grap1() / Grap2() ============================= */

/**
 * @brief  7-step simple pick-and-place grab sequence (22cm horizontal reach)
 * @note   Horz: +22-22 = 0  ✓
 *         Vert: -18+18 = 0  ✓
 */
void Actions_Grap1(void)
{
    /* ---- 1. Servo init ---- */
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    Servo_SetAngle(SERVO_HATCH_A,   63);
    Servo_SetAngle(SERVO_HATCH_B,   117);
    HAL_Delay(500);

    /* ---- 2. Hatch open ---- */
    Servo_SetAngle(SERVO_HATCH_A, 130);
    Servo_SetAngle(SERVO_HATCH_B, 50);
    HAL_Delay(500);

    /* ---- 3. fwd 22cm → after 5cm: down 18cm ---- */
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_HORIZ, 22 * STEPS_PER_CM,
                        STEPPER_VERT,  18 * STEPS_PER_CM, 5 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 4. Gripper close ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_CLOSE);
    HAL_Delay(500);

    /* ---- 5. up 18cm → after 5cm: back 22cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_VERT,  18 * STEPS_PER_CM,
                        STEPPER_HORIZ, 22 * STEPS_PER_CM, 5 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 6. Gripper open ---- */
    Servo_SetAngle(SERVO_HATCH_A,   67);
    Servo_SetAngle(SERVO_HATCH_B,   113);
    HAL_Delay(400);
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    HAL_Delay(500);

    /* ---- 7. Servo init (hatch close) ---- */
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    Servo_SetAngle(SERVO_HATCH_A,   63);
    Servo_SetAngle(SERVO_HATCH_B,   117);
    HAL_Delay(100);

    /* ---- 8. Wait CH6 return to centre (1400–1600us) to prevent re-trigger ---- */
    while (1)
    {
        if (SBUS_IsValid())
        {
            uint16_t ch6 = SBUS_GetChannel(5);
            if (ch6 > SBUS_CH6_ARM_LOW && ch6 < SBUS_CH6_ARM_HIGH)
                break;
        }
        HAL_Delay(10);
    }
}

/**
 * @brief  7-step pick-and-place grab sequence (27cm horizontal reach)
 * @note   Same as Grap1 but with extended 27cm horizontal travel.
 *         Horz: +27-27 = 0  ✓
 *         Vert: -18+18 = 0  ✓
 */
void Actions_Grap2(void)
{
    /* ---- 1. Servo init ---- */
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    Servo_SetAngle(SERVO_HATCH_A,   63);
    Servo_SetAngle(SERVO_HATCH_B,   117);
    HAL_Delay(500);

    /* ---- 2. Hatch open ---- */
    Servo_SetAngle(SERVO_HATCH_A, 130);
    Servo_SetAngle(SERVO_HATCH_B, 50);
    HAL_Delay(500);

    /* ---- 3. fwd 27cm → after 5cm: down 18cm ---- */
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_HORIZ, 27 * STEPS_PER_CM,
                        STEPPER_VERT,  18 * STEPS_PER_CM, 5 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 4. Gripper close ---- */
    Servo_SetAngle(SERVO_GRIPPER, ANGLE_GRIPPER_CLOSE);
    HAL_Delay(500);

    /* ---- 5. up 18cm → after 5cm: back 27cm ---- */
    Stepper_SetDir(STEPPER_VERT, STEP_DIR_FORWARD);
    Stepper_Enable(STEPPER_VERT, STEP_ENA_ON);
    Stepper_SetDir(STEPPER_HORIZ, STEP_DIR_REVERSE);
    Stepper_Enable(STEPPER_HORIZ, STEP_ENA_ON);
    Stepper_MoveOverlap(STEPPER_VERT,  18 * STEPS_PER_CM,
                        STEPPER_HORIZ, 27 * STEPS_PER_CM, 5 * STEPS_PER_CM,
                        STEP_START_DELAY_US, STEP_TARGET_DELAY_US, STEP_ACCEL_STEPS);
    HAL_Delay(300);

    /* ---- 6. Gripper open ---- */
    Servo_SetAngle(SERVO_HATCH_A,   67);
    Servo_SetAngle(SERVO_HATCH_B,   113);
    HAL_Delay(400);
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    HAL_Delay(500);

    /* ---- 7. Servo init (hatch close) ---- */
    Servo_SetAngle(SERVO_GRIPPER,   ANGLE_GRIPPER_OPEN);
    Servo_SetAngle(SERVO_ARM_FRONT, 90);
    Servo_SetAngle(SERVO_HATCH_A,   63);
    Servo_SetAngle(SERVO_HATCH_B,   117);
    HAL_Delay(100);

    /* ---- 8. Wait CH6 return to centre (1400–1600us) to prevent re-trigger ---- */
    while (1)
    {
        if (SBUS_IsValid())
        {
            uint16_t ch6 = SBUS_GetChannel(5);
            if (ch6 > SBUS_CH6_ARM_LOW && ch6 < SBUS_CH6_ARM_HIGH)
                break;
        }
        HAL_Delay(10);
    }
}
