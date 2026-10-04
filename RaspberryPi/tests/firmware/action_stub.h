/* Host-only hardware boundary for the real actions.c state machine. */
#ifndef ACTION_TEST_STUB_H
#define ACTION_TEST_STUB_H
#include <stdint.h>
#define ACK_OK 0
#define ACK_ERR_PARAM 1
#define ACK_ERR_BUSY 2
#define STEPPER_COUNT 2
#define STEPPER_HORIZ 0
#define STEPPER_VERT 1
#define STEP_DIR_FORWARD 0
#define STEP_DIR_REVERSE 1
#define CM_TO_STEPS(cm) ((uint32_t)((cm) * 400))
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
void Servo_HomeAll(void);
void Servo_SetAngle(uint8_t id, uint8_t angle);
void Servo_SetAngleTenth(uint8_t id, uint16_t angle);
void Suction_PumpOn(void);
void Suction_Release(void);
void Suction_AllOff(void);
void Stepper_Stop(uint8_t motor);
uint8_t Stepper_IsBusy(uint8_t motor);
int32_t Stepper_GetPosition(uint8_t motor);
void Stepper_StartMove(uint8_t, uint8_t, uint32_t, uint16_t, uint16_t, uint16_t);
void Stepper_StartMoveOverlap(uint8_t, uint32_t, uint8_t, uint8_t, uint32_t,
                              uint8_t, uint32_t, uint16_t, uint16_t, uint16_t);
void Stepper_StartMoveOverlap2(uint8_t, uint32_t, uint8_t, uint8_t, uint32_t,
                               uint8_t, uint32_t, uint8_t, uint32_t,
                               uint16_t, uint16_t, uint16_t);
void Stepper_StartMoveOverlap3(uint8_t, uint32_t, uint8_t, uint32_t, uint8_t,
                               uint8_t, uint32_t, uint8_t, uint32_t, uint32_t,
                               uint16_t, uint16_t, uint16_t);
#endif
