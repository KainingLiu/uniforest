/**
 ******************************************************************************
 * @file    protocol.h
 * @brief   UART7 communication protocol between STM32 (A-board) and Raspberry Pi
 *          - 115200 bps, 8N1, PE8=TX(AF8), PE7=RX(AF8)
 *          - Binary framed protocol with CRC16
 *          - Command dispatch + telemetry streaming
 ******************************************************************************
 */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= Protocol Constants =============================== */

#define PROTO_SYNC              0xAAu
#define PROTO_MAX_DATA_LEN       80u     /* max payload per frame */
#define PROTO_RX_BUF_SIZE        256u    /* ring buffer for incoming bytes */
#define PROTO_TIMEOUT_MS         200u    /* comm loss → emergency stop */

/* ===================== Command IDs (Pi → STM32) ============================ */

#define CMD_PING                0x01
#define CMD_EMERGENCY_STOP      0x02

#define CMD_CHASSIS_SPEED       0x10   /* 4×int16 RPM targets */
#define CMD_CHASSIS_TORQUE      0x11   /* 4×int16 raw torque */
#define CMD_CHASSIS_PID_SPEED   0x12   /* motor_id + 5×float */
#define CMD_CHASSIS_PID_POS     0x13   /* motor_id + 5×float */
#define CMD_CHASSIS_PID_RESET   0x14   /* motor_id */

#define CMD_SERVO_ANGLE         0x20   /* servo_id + angle_deg */
#define CMD_SERVO_HOME          0x21   /* no data */
#define CMD_SERVO_ANGLE_ALL     0x22   /* 4×uint16 angles */

#define CMD_STEPPER_MOVE        0x30   /* motor + dir + steps(4B) */
#define CMD_STEPPER_STOP        0x31   /* motor */
#define CMD_STEPPER_PARAMS      0x32   /* start_delay + target_delay + accel */
#define CMD_STEPPER_MOVE_DUAL   0x33   /* dual-motor overlap move */
#define CMD_STEPPER_MOVE_DUAL2  0x34   /* dual-motor overlap move with dir change */
#define CMD_STEPPER_SET_POS     0x35   /* motor + pos(4B) */

#define CMD_SET_TELEM_RATE      0x40   /* uint16 rate_hz */

/* =================== Telemetry / Response IDs (STM32 → Pi) ================= */

#define TELEM_FULL              0x80   /* full telemetry batch (80 bytes) */
#define TELEM_ACK               0x81   /* command ACK */
#define TELEM_PONG              0x82   /* PING response */

/* ===================== ACK Status Codes ==================================== */

#define ACK_OK                  0x00
#define ACK_ERR_PARAM           0x01   /* invalid parameter */
#define ACK_ERR_BUSY            0x02   /* motor busy */
#define ACK_ERR_CRC             0x03   /* CRC mismatch */
#define ACK_ERR_UNKNOWN_CMD     0x04   /* unrecognized command */

/* ===================== Motor Count Constants ================================ */

#define M3508_COUNT              4
#define SERVO_COUNT              4
#define STEPPER_COUNT            2

/* ========================== Frame Structure ================================= */

/**
 * @brief Parsed protocol frame (inbound or outbound)
 */
typedef struct {
    uint8_t  cmd;                         /* command / telemetry type */
    uint8_t  len;                         /* frame length from cmd to end of CRC */
    uint8_t  seq;                         /* sequence number */
    uint8_t  data[PROTO_MAX_DATA_LEN];    /* payload */
    uint8_t  data_len;                    /* actual payload length */
} ProtoFrame_t;

/* ===================== Telemetry Data Structures ============================ */

/**
 * @brief Per-motor telemetry (7 bytes, packed)
 */
typedef struct __attribute__((packed)) {
    uint16_t angle;          /* raw encoder angle 0-8191 */
    int16_t  speed_rpm;      /* rotational speed RPM */
    int16_t  torque_current; /* actual torque current */
    uint8_t  temperature;    /* motor temperature ℃ */
} MotorTelem_t;

/**
 * @brief Full telemetry batch (64 bytes, packed)
 */
typedef struct __attribute__((packed)) {
    MotorTelem_t motor[4];   /* 28B: motor feedback */
    float   yaw_deg;         /*  4B: IMU yaw angle ±180° */
    float   yaw_rate_ds;     /*  4B: IMU yaw rate °/s */
    uint16_t rc_channels[6]; /* 12B: RC CH1-CH6 raw (0-2047) */
    uint8_t  stepper_busy;   /*  1B: bit0=H, bit1=V */
    uint8_t  reserved[3];    /*  3B */
    uint32_t uptime_ms;      /*  4B: system uptime */
    int32_t  stepper_pos[2]; /*  8B: cumulative step positions */
    int32_t  motor_pos[4];   /* 16B: cumulative M3508 encoder counts */
} TelemBatch_t;

_Static_assert(sizeof(TelemBatch_t) == 80u,
               "TelemBatch_t wire payload must remain 80 bytes");

/* ========================== Public API ==================================== */

/**
 * @brief  Initialize UART7 and protocol state machine
 * @note   PE8=TX(AF8), PE7=RX(AF8), 115200-8N1
 *         Interrupt-driven RX, blocking TX
 */
void Protocol_Init(void);

/**
 * @brief  Poll for received frames and dispatch commands
 * @note   Call from main loop. Non-blocking.
 *         Returns immediately if no complete frame is available.
 */
void Protocol_RxPoll(void);

/**
 * @brief  Send a protocol frame to the Raspberry Pi
 * @param  cmd       frame type (CMD_* or TELEM_*)
 * @param  seq       sequence number (echo received seq for ACK/PONG)
 * @param  data      payload buffer
 * @param  data_len  payload length in bytes (0..PROTO_MAX_DATA_LEN)
 */
void Protocol_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t data_len);

/**
 * @brief  Send an ACK frame
 * @param  echoed_cmd  the command being acknowledged
 * @param  seq         sequence number from the command
 * @param  status      ACK_OK / ACK_ERR_*
 */
void Protocol_SendAck(uint8_t echoed_cmd, uint8_t seq, uint8_t status);

/**
 * @brief  Pack and send full telemetry batch
 * @note   Call at configured telemetry rate (default off until SET_TELEM_RATE)
 */
void Protocol_SendTelemetry(void);

/**
 * @brief  Check if communication with Pi is alive
 * @retval 1 if last valid frame was within PROTO_TIMEOUT_MS, 0 otherwise
 */
uint8_t Protocol_IsAlive(void);

/**
 * @brief  Get elapsed ms since last valid frame from Pi
 */
uint32_t Protocol_LastFrameAge(void);

/**
 * @brief  CRC16-CCITT (polynomial 0x1021) over a buffer
 */
uint16_t Protocol_CRC16(const uint8_t *data, uint8_t len);

/**
 * @brief  Get UART7 handle (for ISR use)
 */
UART_HandleTypeDef *Protocol_GetUART(void);

/**
 * @brief  Feed one byte from ISR into the RX ring buffer
 */
void Protocol_ISR_FeedByte(uint8_t byte);

/**
 * @brief  Check telemetry timer and send if due (call from main loop)
 */
void Protocol_TelemTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H__ */
