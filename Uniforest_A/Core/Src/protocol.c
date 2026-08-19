/**
 ******************************************************************************
 * @file    protocol.c
 * @brief   UART7 protocol stack — frame encode/decode, CRC16, command dispatch
 *
 *  UART7: PE8=TX(AF8), PE7=RX(AF8), 115200 bps 8N1
 *  RX: interrupt-driven byte-at-a-time → ring buffer → frame parser
 *  TX: blocking HAL_UART_Transmit (frames are short, baud is high)
 *
 *  Frame format (no escaping — sync-byte scanning):
 *    | SYNC(0xAA) | CMD(1) | LEN(1) | SEQ(1) | DATA(N) | CRC16(2) |
 *    LEN = 5 + N,  CRC covers CMD..DATA inclusive (LEN-2 bytes).
 ******************************************************************************
 */

#include "protocol.h"
#include "motor3508.h"
#include "servo.h"
#include "stepper.h"
#include "imu.h"
#include "remote_control.h"
#include <string.h>

/* ============================ Private Constants ============================= */

#define UART7_BAUDRATE          115200u
#define TX_TIMEOUT_MS           10u

/* ============================ UART7 Handle ================================== */

static UART_HandleTypeDef huart7;

/* ========================= RX Ring Buffer =================================== */

static volatile uint8_t  rx_ring[PROTO_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;   /* ISR writes here */
static volatile uint16_t rx_tail = 0;   /* parser reads from here */

/* ========================== Frame Parser State ============================== */

typedef enum {
    PARSE_WAIT_SYNC = 0,
    PARSE_WAIT_CMD,
    PARSE_WAIT_LEN,
    PARSE_WAIT_SEQ,
    PARSE_DATA,
    PARSE_CRC1,
    PARSE_CRC2,
} ParseState_t;

static ParseState_t g_parse_state = PARSE_WAIT_SYNC;
static ProtoFrame_t g_rx_frame;          /* frame being assembled */
static uint8_t      g_rx_data_idx;       /* index into g_rx_frame.data[] */
static uint8_t      g_rx_need;           /* bytes remaining for current state */

/* ========================== Telemetry State ================================= */

static uint16_t g_telem_rate_hz  = 0;    /* 0 = off */
static uint32_t g_last_telem_ms  = 0;
static uint8_t  g_tx_seq         = 0;
static uint32_t g_last_rx_ms     = 0;
static uint8_t  g_comm_alive     = 0;

/* ============================ CRC16 ========================================= */

/* CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF, no reflect, no xor-out) */
uint16_t Protocol_CRC16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000u)
                crc = (crc << 1) ^ 0x1021u;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/* ======================== UART7 Initialization ============================== */

/**
 * @brief  Configure PE8(TX/AF8) and PE7(RX/AF8) for UART7
 */
static void MX_UART7_Init(void)
{
    /* ---- Clocks ---- */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();

    /* ---- GPIO: PE8=TX(AF8), PE7=RX(AF8) ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_UART7;

    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_7;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* ---- UART7: 115200-8N1 ---- */
    memset(&huart7, 0, sizeof(huart7));
    huart7.Instance          = UART7;
    huart7.Init.BaudRate     = UART7_BAUDRATE;
    huart7.Init.WordLength   = UART_WORDLENGTH_8B;
    huart7.Init.StopBits     = UART_STOPBITS_1;
    huart7.Init.Parity       = UART_PARITY_NONE;
    huart7.Init.Mode         = UART_MODE_TX_RX;
    huart7.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_8;
    HAL_UART_Init(&huart7);

    /* ---- Enable RXNE interrupt ---- */
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);

    /* Keep UART RX above the 100 kHz stepper tick so command bytes cannot
     * overrun the peripheral's one-byte data register. */
    HAL_NVIC_SetPriority(UART7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
}

/* ======================== Ring Buffer Helpers =============================== */

/**
 * @brief  Push a byte into the RX ring buffer (called from ISR)
 * @retval 1 on success, 0 on overflow
 */
static inline uint8_t ring_push(uint8_t byte)
{
    uint16_t next = (rx_head + 1) % PROTO_RX_BUF_SIZE;
    if (next == rx_tail) return 0;   /* overflow */
    rx_ring[rx_head] = byte;
    rx_head = next;
    return 1;
}

/**
 * @brief  Pop a byte from the RX ring buffer (called from main loop)
 * @retval 1 on success, 0 if buffer empty
 */
static inline uint8_t ring_pop(uint8_t *byte)
{
    if (rx_head == rx_tail) return 0;
    *byte = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % PROTO_RX_BUF_SIZE;
    return 1;
}

static inline uint8_t ring_empty(void)
{
    return (rx_head == rx_tail);
}

/* ======================== Frame Parser ====================================== */

/* Forward declaration */
static void Protocol_Dispatch(const ProtoFrame_t *f);

/**
 * @brief  Feed one byte into the frame parser (called from main loop)
 * @note   On complete valid frame, dispatches the command.
 *          On CRC error, sends ACK_ERR_CRC and resets.
 */
static void Protocol_ParseByte(uint8_t byte)
{
    switch (g_parse_state)
    {
    case PARSE_WAIT_SYNC:
        if (byte == PROTO_SYNC)
        {
            memset(&g_rx_frame, 0, sizeof(g_rx_frame));
            g_parse_state = PARSE_WAIT_CMD;
        }
        break;

    case PARSE_WAIT_CMD:
        g_rx_frame.cmd = byte;
        g_parse_state = PARSE_WAIT_LEN;
        break;

    case PARSE_WAIT_LEN:
        g_rx_frame.len = byte;
        if (byte < 5 || byte > PROTO_MAX_DATA_LEN + 5)
        {
            /* invalid length — discard and re-sync */
            g_parse_state = PARSE_WAIT_SYNC;
        }
        else
        {
            g_rx_data_idx = 0;
            g_parse_state = PARSE_WAIT_SEQ;
        }
        break;

    case PARSE_WAIT_SEQ:
        g_rx_frame.seq = byte;
        /* Need: LEN - CMD(1) - LEN(1) - SEQ(1) - CRC(2) = data_len */
        g_rx_need = g_rx_frame.len - 5;
        if (g_rx_need == 0)
        {
            g_parse_state = PARSE_CRC1;
        }
        else
        {
            g_parse_state = PARSE_DATA;
        }
        break;

    case PARSE_DATA:
        g_rx_frame.data[g_rx_data_idx++] = byte;
        g_rx_need--;
        if (g_rx_need == 0)
        {
            g_parse_state = PARSE_CRC1;
        }
        break;

    case PARSE_CRC1:
        /* CRC low byte — stash temporarily */
        g_rx_frame.data[g_rx_data_idx] = byte;
        g_parse_state = PARSE_CRC2;
        break;

    case PARSE_CRC2:
    {
        /* CRC high byte — validate */
        uint16_t rx_crc = (uint16_t)byte << 8
                        | (uint16_t)g_rx_frame.data[g_rx_data_idx];

        /* Build verification buffer: CMD + LEN + SEQ + DATA */
        uint8_t crc_buf[PROTO_MAX_DATA_LEN + 3];
        uint8_t crc_len = 3 + (g_rx_frame.len - 5);  /* header + payload */
        crc_buf[0] = g_rx_frame.cmd;
        crc_buf[1] = g_rx_frame.len;
        crc_buf[2] = g_rx_frame.seq;
        if (crc_len > 3)
            memcpy(crc_buf + 3, g_rx_frame.data, crc_len - 3);

        uint16_t calc_crc = Protocol_CRC16(crc_buf, crc_len);

        if (rx_crc == calc_crc)
        {
            g_rx_frame.data_len = g_rx_frame.len - 5;
            g_last_rx_ms = HAL_GetTick();
            g_comm_alive = 1;

            /* Dispatch the command (implemented below) */
            Protocol_Dispatch(&g_rx_frame);
        }
        else
        {
            /* CRC mismatch — send NAK */
            Protocol_SendAck(g_rx_frame.cmd, g_rx_frame.seq, ACK_ERR_CRC);
        }
        g_parse_state = PARSE_WAIT_SYNC;
        break;
    }
    }
}

/* ======================== Frame Sender ====================================== */

/**
 * @brief  Send a raw frame over UART7
 */
void Protocol_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *data, uint8_t data_len)
{
    uint8_t buf[PROTO_MAX_DATA_LEN + 6];  /* SYNC + CMD + LEN + SEQ + DATA + CRC */
    uint8_t idx = 0;
    uint8_t len = 5 + data_len;           /* CMD + LEN + SEQ + DATA + CRC */

    buf[idx++] = PROTO_SYNC;
    buf[idx++] = cmd;
    buf[idx++] = len;
    buf[idx++] = seq;

    if (data_len > 0 && data != NULL)
    {
        memcpy(buf + idx, data, data_len);
        idx += data_len;
    }

    /* CRC over CMD..DATA (bytes buf[1] through buf[idx-1]) */
    uint16_t crc = Protocol_CRC16(buf + 1, idx - 1);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    HAL_UART_Transmit(&huart7, buf, idx, TX_TIMEOUT_MS);
}

void Protocol_SendAck(uint8_t echoed_cmd, uint8_t seq, uint8_t status)
{
    uint8_t data[3];
    data[0] = echoed_cmd;
    data[1] = status;
    data[2] = 0;  /* detail — reserved */
    Protocol_SendFrame(TELEM_ACK, seq, data, 3);
}

/* ======================== Telemetry Packing ================================= */

static inline void put_u16_be(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)value;
}

static inline void put_u32_be(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)value;
}

static inline void put_float_be(uint8_t *dst, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    put_u32_be(dst, raw);
}

/**
 * @brief  Pack and send the full telemetry batch (80 bytes payload)
 */
void Protocol_SendTelemetry(void)
{
    TelemBatch_t telem;
    uint8_t payload[sizeof(TelemBatch_t)];
    uint8_t offset = 0;
    memset(&telem, 0, sizeof(telem));

    /* ---- Motor feedback ---- */
    for (uint8_t i = 0; i < M3508_COUNT; i++)
    {
        uint16_t angle;
        int16_t speed_rpm;
        int16_t torque_current;
        uint8_t temperature;
        Motor3508_GetFeedback(i, &angle, &speed_rpm,
                                 &torque_current, &temperature);
        telem.motor[i].angle = angle;
        telem.motor[i].speed_rpm = speed_rpm;
        telem.motor[i].torque_current = torque_current;
        telem.motor[i].temperature = temperature;
        telem.motor_pos[i] = Motor3508_GetCumulativePosition(i);
    }

    /* ---- IMU ---- */
    telem.yaw_deg     = IMU_GetYaw();
    telem.yaw_rate_ds = IMU_GetYawRate();

    /* ---- RC channels ---- */
    for (uint8_t i = 0; i < 6; i++)
        telem.rc_channels[i] = SBUS_GetChannel(i);

    /* ---- Stepper status ---- */
    telem.stepper_busy = 0;
    if (Stepper_IsBusy(STEPPER_HORIZ)) telem.stepper_busy |= (1u << 0);
    if (Stepper_IsBusy(STEPPER_VERT))  telem.stepper_busy |= (1u << 1);

    for (uint8_t i = 0; i < STEPPER_COUNT; i++)
        telem.stepper_pos[i] = (int32_t)Stepper_GetPosition(i);

    /* ---- System ---- */
    telem.uptime_ms = HAL_GetTick();

    /* The STM32 is little-endian, while the wire protocol is big-endian. */
    for (uint8_t i = 0; i < M3508_COUNT; i++)
    {
        put_u16_be(payload + offset, telem.motor[i].angle);
        offset += 2;
        put_u16_be(payload + offset, (uint16_t)telem.motor[i].speed_rpm);
        offset += 2;
        put_u16_be(payload + offset, (uint16_t)telem.motor[i].torque_current);
        offset += 2;
        payload[offset++] = telem.motor[i].temperature;
    }

    put_float_be(payload + offset, telem.yaw_deg);
    offset += 4;
    put_float_be(payload + offset, telem.yaw_rate_ds);
    offset += 4;

    for (uint8_t i = 0; i < 6; i++)
    {
        put_u16_be(payload + offset, telem.rc_channels[i]);
        offset += 2;
    }

    payload[offset++] = telem.stepper_busy;
    payload[offset++] = 0;
    payload[offset++] = 0;
    payload[offset++] = 0;

    put_u32_be(payload + offset, telem.uptime_ms);
    offset += 4;
    for (uint8_t i = 0; i < STEPPER_COUNT; i++)
    {
        put_u32_be(payload + offset, (uint32_t)telem.stepper_pos[i]);
        offset += 4;
    }
    for (uint8_t i = 0; i < M3508_COUNT; i++)
    {
        put_u32_be(payload + offset, (uint32_t)telem.motor_pos[i]);
        offset += 4;
    }

    Protocol_SendFrame(TELEM_FULL, 0, payload, offset);
}

/* ======================== Command Dispatch ================================== */

/**
 * @brief  Parse a float from big-endian bytes
 */
static inline float parse_float_be(const uint8_t *b)
{
    uint32_t raw = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16)
                 | ((uint32_t)b[2] <<  8) |  (uint32_t)b[3];
    float f;
    memcpy(&f, &raw, 4);
    return f;
}

/**
 * @brief  Parse int16 from big-endian bytes
 */
static inline int16_t parse_i16_be(const uint8_t *b)
{
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

/**
 * @brief  Parse uint16 from big-endian bytes
 */
static inline uint16_t parse_u16_be(const uint8_t *b)
{
    return ((uint16_t)b[0] << 8) | b[1];
}

/**
 * @brief  Parse int32 from big-endian bytes
 */
static inline int32_t parse_i32_be(const uint8_t *b)
{
    return (int32_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16)
                   | ((uint32_t)b[2] <<  8) |  (uint32_t)b[3]);
}

/**
 * @brief  Dispatch a valid inbound command to the appropriate handler
 */
static void Protocol_Dispatch(const ProtoFrame_t *f)
{
    const uint8_t *d = f->data;
    uint8_t status = ACK_OK;

    switch (f->cmd)
    {
    /* ---- Ping ---- */
    case CMD_PING:
    {
        uint32_t uptime = HAL_GetTick();
        uint8_t pong[4];
        pong[0] = (uint8_t)(uptime >> 24);
        pong[1] = (uint8_t)(uptime >> 16);
        pong[2] = (uint8_t)(uptime >>  8);
        pong[3] = (uint8_t)(uptime);
        Protocol_SendFrame(TELEM_PONG, f->seq, pong, 4);
        return;  /* PONG is the ACK — no separate ACK needed */
    }

    /* ---- Emergency Stop ---- */
    case CMD_EMERGENCY_STOP:
        Motor3508_StopAll();
        Stepper_Stop(STEPPER_HORIZ);
        Stepper_Stop(STEPPER_VERT);
        break;

    /* ---- Chassis: Speed ---- */
    case CMD_CHASSIS_SPEED:
        if (f->data_len >= 8)
        {
            int16_t rpm[4];
            for (int i = 0; i < 4; i++)
                rpm[i] = parse_i16_be(d + i * 2);
            Motor3508_SetAllSpeeds(rpm);
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Chassis: Raw Torque ---- */
    case CMD_CHASSIS_TORQUE:
        if (f->data_len >= 8)
        {
            int16_t tq[4];
            for (int i = 0; i < 4; i++)
                tq[i] = parse_i16_be(d + i * 2);
            Motor3508_SendTorques(tq);
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Chassis: Speed PID Params ---- */
    case CMD_CHASSIS_PID_SPEED:
        if (f->data_len >= 21)
        {
            uint8_t mid = d[0];
            float kp   = parse_float_be(d + 1);
            float ki   = parse_float_be(d + 5);
            float kd   = parse_float_be(d + 9);
            float ilim = parse_float_be(d + 13);
            float olim = parse_float_be(d + 17);
            Motor3508_SetSpeedPID(mid, kp, ki, kd, ilim, olim);
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Chassis: Position PID Params ---- */
    case CMD_CHASSIS_PID_POS:
        if (f->data_len >= 21)
        {
            uint8_t mid = d[0];
            float kp   = parse_float_be(d + 1);
            float ki   = parse_float_be(d + 5);
            float kd   = parse_float_be(d + 9);
            float ilim = parse_float_be(d + 13);
            float olim = parse_float_be(d + 17);
            Motor3508_SetPosPID(mid, kp, ki, kd, ilim, olim);
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Chassis: PID Reset ---- */
    case CMD_CHASSIS_PID_RESET:
        if (f->data_len >= 1)
            Motor3508_ResetPID(d[0]);
        else
            status = ACK_ERR_PARAM;
        break;

    /* ---- Servo: Single Angle ---- */
    case CMD_SERVO_ANGLE:
        if (f->data_len >= 2)
        {
            uint8_t sid = d[0];
            uint8_t ang = d[1];
            if (sid < SERVO_COUNT && ang <= 180)
                Servo_SetAngle(sid, ang);
            else
                status = ACK_ERR_PARAM;
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Servo: Home All ---- */
    case CMD_SERVO_HOME:
        Servo_HomeAll();
        break;

    /* ---- Servo: All Angles ---- */
    case CMD_SERVO_ANGLE_ALL:
        if (f->data_len >= 4)
        {
            for (int i = 0; i < 4; i++)
            {
                if (d[i] <= 180)
                    Servo_SetAngle(i, d[i]);
            }
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Stepper: Single Move ---- */
    case CMD_STEPPER_MOVE:
        if (f->data_len >= 6)
        {
            uint8_t  motor = d[0];
            uint8_t  dir   = d[1];
            uint32_t steps = (uint32_t)parse_i32_be(d + 2);
            if (motor < STEPPER_COUNT)
                Stepper_StartMove(motor, dir, steps, 0, 0, 0);
            else
                status = ACK_ERR_PARAM;
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Stepper: Stop ---- */
    case CMD_STEPPER_STOP:
        if (f->data_len >= 1)
        {
            if (d[0] < STEPPER_COUNT)
                Stepper_Stop(d[0]);
            else
                status = ACK_ERR_PARAM;
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Stepper: Speed Params ---- */
    case CMD_STEPPER_PARAMS:
        if (f->data_len >= 6)
        {
            uint16_t start  = parse_u16_be(d);
            uint16_t target = parse_u16_be(d + 2);
            uint16_t accel  = parse_u16_be(d + 4);
            Stepper_SetParams(start, target, accel);
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Stepper: Dual Move (Overlap) ---- */
    case CMD_STEPPER_MOVE_DUAL:
        if (f->data_len >= 22)
        {
            uint8_t  m1       = d[0];
            uint32_t steps1   = (uint32_t)parse_i32_be(d + 1);
            uint8_t  dir1     = d[5];
            uint8_t  m2       = d[6];
            uint32_t steps2   = (uint32_t)parse_i32_be(d + 7);
            uint8_t  dir2     = d[11];
            uint32_t m2_off   = (uint32_t)parse_i32_be(d + 12);
            uint16_t start_d  = parse_u16_be(d + 16);
            uint16_t target_d = parse_u16_be(d + 18);
            uint16_t accel_s  = parse_u16_be(d + 20);
            if (m1 < STEPPER_COUNT && m2 < STEPPER_COUNT)
                Stepper_StartMoveOverlap(m1, steps1, dir1,
                                        m2, steps2, dir2, m2_off,
                                        start_d, target_d, accel_s);
            else
                status = ACK_ERR_PARAM;
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Stepper: Dual Move with Direction Change ---- */
    case CMD_STEPPER_MOVE_DUAL2:
        if (f->data_len >= 27)
        {
            uint8_t  m_cont    = d[0];
            uint32_t steps_cont= (uint32_t)parse_i32_be(d + 1);
            uint8_t  dir_cont  = d[5];
            uint8_t  m_ph      = d[6];
            uint32_t steps_ph1 = (uint32_t)parse_i32_be(d + 7);
            uint8_t  dir_ph1   = d[11];
            uint32_t steps_ph2 = (uint32_t)parse_i32_be(d + 12);
            uint8_t  dir_ph2   = d[16];
            uint32_t ph2_off   = (uint32_t)parse_i32_be(d + 17);
            uint16_t start_d   = parse_u16_be(d + 21);
            uint16_t target_d  = parse_u16_be(d + 23);
            uint16_t accel_s   = parse_u16_be(d + 25);
            if (m_cont < STEPPER_COUNT && m_ph < STEPPER_COUNT)
                Stepper_StartMoveOverlap2(m_cont, steps_cont, dir_cont,
                                         m_ph, steps_ph1, dir_ph1,
                                         steps_ph2, dir_ph2, ph2_off,
                                         start_d, target_d, accel_s);
            else
                status = ACK_ERR_PARAM;
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Stepper: Cross-Triggered Three-Segment Move ---- */
    case CMD_STEPPER_MOVE_DUAL3:
        if (f->data_len >= 31)
        {
            uint8_t  m_lead      = d[0];
            uint32_t steps_lead1 = (uint32_t)parse_i32_be(d + 1);
            uint8_t  dir_lead1   = d[5];
            uint32_t steps_lead2 = (uint32_t)parse_i32_be(d + 6);
            uint8_t  dir_lead2   = d[10];
            uint8_t  m_other     = d[11];
            uint32_t steps_other = (uint32_t)parse_i32_be(d + 12);
            uint8_t  dir_other   = d[16];
            uint32_t other_off   = (uint32_t)parse_i32_be(d + 17);
            uint32_t lead2_off   = (uint32_t)parse_i32_be(d + 21);
            uint16_t start_d     = parse_u16_be(d + 25);
            uint16_t target_d    = parse_u16_be(d + 27);
            uint16_t accel_s     = parse_u16_be(d + 29);
            if (m_lead < STEPPER_COUNT && m_other < STEPPER_COUNT &&
                m_lead != m_other && steps_lead1 > 0 &&
                steps_lead2 > 0 && steps_other > 0 &&
                other_off <= steps_lead1 && lead2_off <= steps_other)
                Stepper_StartMoveOverlap3(
                    m_lead, steps_lead1, dir_lead1,
                    steps_lead2, dir_lead2,
                    m_other, steps_other, dir_other,
                    other_off, lead2_off,
                    start_d, target_d, accel_s);
            else
                status = ACK_ERR_PARAM;
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Stepper: Set Position ---- */
    case CMD_STEPPER_SET_POS:
        if (f->data_len >= 5)
        {
            uint8_t  motor = d[0];
            int32_t  pos   = parse_i32_be(d + 1);
            if (motor < STEPPER_COUNT)
                Stepper_SetPosition(motor, pos);
            else
                status = ACK_ERR_PARAM;
        }
        else status = ACK_ERR_PARAM;
        break;

    /* ---- Telemetry Rate ---- */
    case CMD_SET_TELEM_RATE:
        if (f->data_len >= 2)
        {
            g_telem_rate_hz = parse_u16_be(d);
            g_last_telem_ms = HAL_GetTick();  /* reset timer */
        }
        else status = ACK_ERR_PARAM;
        break;

    default:
        status = ACK_ERR_UNKNOWN_CMD;
        break;
    }

    /* Send ACK for all commands except PING (which sends PONG instead) */
    Protocol_SendAck(f->cmd, f->seq, status);
}

/* ======================== Public API ======================================== */

void Protocol_Init(void)
{
    /* Reset state */
    memset((void *)rx_ring, 0, sizeof(rx_ring));
    rx_head = 0;
    rx_tail = 0;
    g_parse_state = PARSE_WAIT_SYNC;
    memset(&g_rx_frame, 0, sizeof(g_rx_frame));
    g_telem_rate_hz = 0;
    g_last_telem_ms = 0;
    g_tx_seq = 0;
    g_last_rx_ms = 0;
    g_comm_alive = 0;

    MX_UART7_Init();
}

void Protocol_RxPoll(void)
{
    uint8_t byte;
    while (ring_pop(&byte))
    {
        Protocol_ParseByte(byte);
    }
}

uint8_t Protocol_IsAlive(void)
{
    if (!g_comm_alive) return 0;
    if ((HAL_GetTick() - g_last_rx_ms) > PROTO_TIMEOUT_MS)
    {
        g_comm_alive = 0;
        return 0;
    }
    return 1;
}

uint32_t Protocol_LastFrameAge(void)
{
    return HAL_GetTick() - g_last_rx_ms;
}

/**
 * @brief  Check if it's time to send telemetry and send if due
 * @note   Call this from main loop. Respects g_telem_rate_hz.
 */
void Protocol_TelemTick(void)
{
    if (g_telem_rate_hz == 0) return;

    uint32_t interval = 1000u / g_telem_rate_hz;
    if (interval < 5) interval = 5;  /* clamp to 200Hz max */
    uint32_t now = HAL_GetTick();
    if (now - g_last_telem_ms >= interval)
    {
        /* Advance the fixed deadline instead of accumulating the main loop's
         * 1 ms scheduling granularity.  Skip catch-up bursts after a delay of
         * two or more periods. */
        if (now - g_last_telem_ms >= interval * 2u)
            g_last_telem_ms = now;
        else
            g_last_telem_ms += interval;
        Protocol_SendTelemetry();
    }
}

/* ========================= ISR Access ======================================= */

UART_HandleTypeDef *Protocol_GetUART(void)
{
    return &huart7;
}

/**
 * @brief  Feed one byte from ISR into the RX ring buffer
 * @note   Called from UART7_IRQHandler in stm32f4xx_it.c
 */
void Protocol_ISR_FeedByte(uint8_t byte)
{
    ring_push(byte);
}
