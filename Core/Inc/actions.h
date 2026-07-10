/**
 ******************************************************************************
 * @file    actions.h
 * @brief   Macro-action sequences: Build() and Grap()
 *          Ported from uniforest_0616.
 *
 *  Build(): 24-step pick-and-place cycle for building a structure.
 *  Grap():   7-step simple pick-and-place for grabbing an object.
 *
 *  Both rely on servo.h and stepper.h — they do NOT touch chassis motors.
 ******************************************************************************
 */

#ifndef __ACTIONS_H__
#define __ACTIONS_H__

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== Semantic Angle Aliases ============================= */
/*  Convenience — same values as the home/action constants in servo.h         */

#define ANGLE_GRIPPER_OPEN          83
#define ANGLE_GRIPPER_CLOSE         102
#define ANGLE_ARM_FRONT_DOWN        90
#define ANGLE_ARM_FRONT_UP          0
#define ANGLE_HATCH_A_CLOSED        63
#define ANGLE_HATCH_A_OPEN          130
#define ANGLE_HATCH_B_CLOSED        117
#define ANGLE_HATCH_B_OPEN          50

/* ========================= Public Functions =============================== */

/**
 * @brief  24-step Build action
 * @note   Blocking — may take several minutes to complete.
 *         Stops immediately if any step fails.
 */
void Actions_Build(void);

/**
 * @brief  7-step Grab action
 * @note   Blocking — may take tens of seconds.
 */
void Actions_Grap(void);

/**
 * @brief  Quick servo home (no stepper movement)
 *         S1=84°, S2=90°, S3=63°, S4=117°
 *         Waits 500 ms for servos to reach position.
 */
void Actions_ServoHome(void);

/**
 * @brief  Open both hatches (S3→130°, S4→50°)
 *         Waits 500 ms.
 */
void Actions_HatchOpen(void);

/**
 * @brief  Close both hatches (S3→63°, S4→117°)
 *         Waits 500 ms.
 */
void Actions_HatchClose(void);

#ifdef __cplusplus
}
#endif

#endif /* __ACTIONS_H__ */
