"""
Command and telemetry data structures for the STM32 ↔ Raspberry Pi protocol.

Frame format:
    | SYNC(0xAA) | CMD(1) | LEN(1) | SEQ(1) | DATA(N) | CRC16(2) |

All multi-byte integers are big-endian.  Floats are IEEE 754 big-endian.
"""

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

PROTO_SYNC = 0xAA
PROTO_MAX_DATA_LEN = 80

# ======================== Command IDs (Pi → STM32) ===========================

CMD_PING             = 0x01
CMD_EMERGENCY_STOP   = 0x02

CMD_CHASSIS_SPEED    = 0x10  # 4×int16 RPM targets
CMD_CHASSIS_TORQUE   = 0x11  # 4×int16 raw torque
CMD_CHASSIS_PID_SPEED= 0x12  # motor_id + 5×float (Kp,Ki,Kd,Ilim,Olim)
CMD_CHASSIS_PID_POS  = 0x13  # motor_id + 5×float
CMD_CHASSIS_PID_RESET= 0x14  # motor_id

CMD_SERVO_ANGLE      = 0x20  # servo_id + angle_deg
CMD_SERVO_HOME       = 0x21  # no data
CMD_SERVO_ANGLE_ALL  = 0x22  # 4×uint8 angles

CMD_STEPPER_MOVE     = 0x30  # motor + dir + steps(4B)
CMD_STEPPER_STOP     = 0x31  # motor
CMD_STEPPER_PARAMS   = 0x32  # start_delay + target_delay + accel
CMD_STEPPER_MOVE_DUAL= 0x33  # dual-motor overlap
CMD_STEPPER_MOVE_DUAL2=0x34  # dual-motor overlap with dir change
CMD_STEPPER_SET_POS  = 0x35  # motor + pos(4B)
CMD_STEPPER_MOVE_DUAL3=0x36  # cross-triggered three-segment overlap

CMD_SET_TELEM_RATE   = 0x40  # uint16 rate_hz

# =================== Telemetry / Response IDs (STM32 → Pi) ===================

TELEM_FULL  = 0x80  # 80-byte telemetry batch
TELEM_ACK   = 0x81  # command acknowledgement
TELEM_PONG  = 0x82  # ping response

# ======================= ACK Status Codes ====================================

ACK_OK           = 0x00
ACK_ERR_PARAM    = 0x01
ACK_ERR_BUSY     = 0x02
ACK_ERR_CRC      = 0x03
ACK_ERR_UNKNOWN  = 0x04

# ===================== Motor / Servo / Stepper Constants =====================

M3508_IDX_TR = 0  # top-right
M3508_IDX_TL = 1  # top-left
M3508_IDX_BL = 2  # bottom-left
M3508_IDX_BR = 3  # bottom-right
M3508_COUNT  = 4

SERVO_GRIPPER   = 0
SERVO_ARM_FRONT = 1
SERVO_HATCH_A   = 2
SERVO_HATCH_B   = 3
SERVO_COUNT     = 4

STEPPER_HORIZ = 0
STEPPER_VERT  = 1

STEP_DIR_FORWARD = 0
STEP_DIR_REVERSE = 1

# ======================= Telemetry Data Structures ============================

@dataclass
class MotorFeedback:
    """Single motor telemetry plus the batch's multi-turn position."""
    angle: int = 0          # raw encoder 0–8191
    speed_rpm: int = 0      # signed RPM
    torque_current: int = 0 # signed raw current
    temperature: int = 0    # ℃
    cumulative_pos: int = 0 # signed multi-turn encoder counts


@dataclass
class TelemBatch:
    """Full telemetry batch (80 bytes on wire)."""
    PAYLOAD_SIZE = 80
    motors: List[MotorFeedback] = field(default_factory=lambda: [MotorFeedback() for _ in range(4)])
    yaw_deg: float = 0.0
    yaw_rate_ds: float = 0.0
    rc_channels: List[int] = field(default_factory=lambda: [0] * 6)
    stepper_busy: int = 0       # bit0=H, bit1=V
    uptime_ms: int = 0
    stepper_pos: List[int] = field(default_factory=lambda: [0, 0])

    @classmethod
    def unpack(cls, data: bytes) -> 'TelemBatch':
        """Parse an 80-byte telemetry batch from wire format."""
        if len(data) < cls.PAYLOAD_SIZE:
            raise ValueError(
                f'telemetry payload too short: {len(data)} < {cls.PAYLOAD_SIZE}')
        t = cls()
        off = 0
        for i in range(4):
            t.motors[i] = MotorFeedback(
                angle         = struct.unpack_from('>H', data, off)[0],
                speed_rpm     = struct.unpack_from('>h', data, off + 2)[0],
                torque_current= struct.unpack_from('>h', data, off + 4)[0],
                temperature   = data[off + 6],
            )
            off += 7
        t.yaw_deg      = struct.unpack_from('>f', data, off)[0]; off += 4
        t.yaw_rate_ds  = struct.unpack_from('>f', data, off)[0]; off += 4
        for i in range(6):
            t.rc_channels[i] = struct.unpack_from('>H', data, off)[0]; off += 2
        t.stepper_busy = data[off]; off += 1
        off += 3  # reserved
        t.uptime_ms   = struct.unpack_from('>I', data, off)[0]; off += 4
        for i in range(2):
            t.stepper_pos[i] = struct.unpack_from('>i', data, off)[0]; off += 4
        for i in range(4):
            t.motors[i].cumulative_pos = struct.unpack_from('>i', data, off)[0]
            off += 4
        return t


@dataclass
class AckFrame:
    """Command acknowledgement."""
    echoed_cmd: int = 0
    status: int = 0

    @classmethod
    def unpack(cls, data: bytes) -> 'AckFrame':
        return cls(echoed_cmd=data[0], status=data[1])


# ======================= Command Encoding Helpers =============================

def encode_ping() -> bytes:
    return b''

def encode_emergency_stop() -> bytes:
    return b''

def encode_chassis_speed(rpm: List[int]) -> bytes:
    """4×int16 big-endian."""
    return struct.pack('>4h', *rpm[:4])

def encode_chassis_torque(torque: List[int]) -> bytes:
    """4×int16 big-endian."""
    return struct.pack('>4h', *torque[:4])

def encode_chassis_pid_speed(motor_id: int, kp: float, ki: float,
                             kd: float, ilim: float, olim: float) -> bytes:
    return struct.pack('>B5f', motor_id, kp, ki, kd, ilim, olim)

def encode_chassis_pid_pos(motor_id: int, kp: float, ki: float,
                           kd: float, ilim: float, olim: float) -> bytes:
    return struct.pack('>B5f', motor_id, kp, ki, kd, ilim, olim)

def encode_chassis_pid_reset(motor_id: int) -> bytes:
    return struct.pack('>B', motor_id)

def encode_servo_angle(servo_id: int, angle_deg: int) -> bytes:
    return struct.pack('>BB', servo_id, angle_deg)

def encode_servo_home() -> bytes:
    return b''

def encode_servo_angle_all(angles: List[int]) -> bytes:
    return bytes(angles[:4])

def encode_stepper_move(motor: int, direction: int, steps: int,
                        start_delay: int = 0, target_delay: int = 0,
                        accel_steps: int = 0) -> bytes:
    """Single motor trapezoidal move.  Delays of 0 use STM32 defaults."""
    return struct.pack('>BBI', motor, direction, steps)

def encode_stepper_stop(motor: int) -> bytes:
    return struct.pack('>B', motor)

def encode_stepper_params(start_delay: int, target_delay: int,
                          accel_steps: int) -> bytes:
    return struct.pack('>3H', start_delay, target_delay, accel_steps)

def encode_stepper_move_dual(m1: int, steps1: int, dir1: int,
                             m2: int, steps2: int, dir2: int,
                             m2_offset: int,
                             start_delay: int, target_delay: int,
                             accel_steps: int) -> bytes:
    """22-byte wire format: BIBBIB I 3H."""
    return struct.pack('>BIBBIB I 3H',
                       m1, steps1, dir1,
                       m2, steps2, dir2,
                       m2_offset,
                       start_delay, target_delay, accel_steps)

def encode_stepper_move_dual2(m_cont: int, steps_cont: int, dir_cont: int,
                              m_ph: int, steps_ph1: int, dir_ph1: int,
                              steps_ph2: int, dir_ph2: int,
                              ph2_offset: int,
                              start_delay: int, target_delay: int,
                              accel_steps: int) -> bytes:
    """27-byte wire format matching STM32 CMD_STEPPER_MOVE_DUAL2 offsets."""
    return struct.pack('>BIBBIBIBI3H',
                       m_cont, steps_cont, dir_cont,
                       m_ph, steps_ph1, dir_ph1,
                       steps_ph2, dir_ph2, ph2_offset,
                       start_delay, target_delay, accel_steps)

def encode_stepper_move_dual3(
        m_lead: int, steps_lead1: int, dir_lead1: int,
        steps_lead2: int, dir_lead2: int,
        m_other: int, steps_other: int, dir_other: int,
        other_offset: int, lead2_offset: int,
        start_delay: int, target_delay: int,
        accel_steps: int) -> bytes:
    """31-byte cross-triggered three-segment overlap command."""
    return struct.pack(
        '>BIBIBBIBII3H',
        m_lead, steps_lead1, dir_lead1,
        steps_lead2, dir_lead2,
        m_other, steps_other, dir_other,
        other_offset, lead2_offset,
        start_delay, target_delay, accel_steps)

def encode_stepper_set_pos(motor: int, pos: int) -> bytes:
    return struct.pack('>Bi', motor, pos)

def encode_set_telem_rate(rate_hz: int) -> bytes:
    return struct.pack('>H', rate_hz)
