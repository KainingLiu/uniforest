"""
UART serial transport layer for STM32 ↔ Raspberry Pi communication.

Implements the binary framed protocol:
    | SYNC(0xAA) | CMD(1) | LEN(1) | SEQ(1) | DATA(N) | CRC16(2) |

Features:
- Background receive thread for non-blocking reads
- Callback dispatch for telemetry and ACK frames
- Automatic CRC verification and error reporting
- Sequence numbering for frame tracking
"""

import serial
import struct
import threading
import time
from typing import Callable, Optional

from utils.crc16 import crc16_ccitt
from protocol.commands import (
    CMD_PING, CMD_EMERGENCY_STOP,
    CMD_CHASSIS_SPEED, CMD_CHASSIS_TORQUE,
    CMD_CHASSIS_PID_SPEED, CMD_CHASSIS_PID_POS, CMD_CHASSIS_PID_RESET,
    CMD_SERVO_ANGLE, CMD_SERVO_HOME, CMD_SERVO_ANGLE_ALL,
    CMD_STEPPER_MOVE, CMD_STEPPER_STOP, CMD_STEPPER_PARAMS,
    CMD_STEPPER_MOVE_DUAL, CMD_STEPPER_MOVE_DUAL2, CMD_STEPPER_SET_POS,
    CMD_STEPPER_MOVE_DUAL3,
    CMD_SET_TELEM_RATE,
    TELEM_FULL, TELEM_ACK, TELEM_PONG,
    ACK_OK, ACK_ERR_CRC,
    TelemBatch, AckFrame, PROTO_SYNC, PROTO_MAX_DATA_LEN,
    encode_chassis_speed, encode_chassis_torque,
    encode_chassis_pid_speed, encode_chassis_pid_pos, encode_chassis_pid_reset,
    encode_servo_angle, encode_servo_angle_all,
    encode_stepper_move, encode_stepper_stop, encode_stepper_params,
    encode_stepper_move_dual, encode_stepper_move_dual2, encode_stepper_set_pos,
    encode_stepper_move_dual3,
    encode_set_telem_rate,
)

class Transport:
    """Serial transport with protocol framing."""

    def __init__(self, port: str = '/dev/ttyAMA0', baudrate: int = 115200,
                 timeout: float = 0.01, debug: bool = False):
        self._port = port
        self._baudrate = baudrate
        self._seq = 0
        self._tx_lock = threading.Lock()
        self._running = False
        self._debug = debug

        # Serial port (opened in connect())
        self._ser: Optional[serial.Serial] = None

        # Receive state
        self._rx_buf = bytearray()
        self._thread: Optional[threading.Thread] = None

        # Callbacks
        self._on_telemetry: Optional[Callable[[TelemBatch], None]] = None
        self._on_ack: Optional[Callable[[AckFrame], None]] = None
        self._on_pong: Optional[Callable[[int], None]] = None

        # Stats
        self.rx_frames = 0
        self.rx_crc_errors = 0
        self.tx_frames = 0

    # ======================== Connection ======================================

    def connect(self) -> bool:
        """Open the serial port and start the receive thread."""
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._timeout if hasattr(self, '_timeout') else 0.01,
                write_timeout=0.5,
            )
            self._running = True
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()
            return True
        except serial.SerialException as e:
            print(f"[Transport] Failed to open {self._port}: {e}")
            return False

    def disconnect(self):
        """Stop the receive thread and close the port."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ======================== Callbacks =======================================

    def on_telemetry(self, cb: Callable[[TelemBatch], None]):
        """Register callback for telemetry batches."""
        self._on_telemetry = cb

    def on_ack(self, cb: Callable[[AckFrame], None]):
        """Register callback for command ACKs."""
        self._on_ack = cb

    def on_pong(self, cb: Callable[[int], None]):
        """Register callback for PONG responses (uptime_ms)."""
        self._on_pong = cb

    # ======================== Send ============================================

    def send(self, cmd: int, data: bytes = b'') -> bool:
        """
        Send a protocol frame to the STM32.

        Args:
            cmd:  Command byte (CMD_* constants)
            data: Payload bytes

        Returns:
            True if sent successfully.
        """
        if not self.connected:
            return False

        with self._tx_lock:
            self._seq = (self._seq + 1) & 0xFF
            data_len = len(data)
            frame_len = 5 + data_len

            # Build frame body (CMD through DATA, for CRC)
            body = struct.pack('>BBB', cmd, frame_len, self._seq) + data
            crc = crc16_ccitt(body)
            frame = bytes([PROTO_SYNC]) + body + struct.pack('<H', crc)

            try:
                self._ser.write(frame)
                self.tx_frames += 1
                return True
            except serial.SerialTimeoutException:
                print("[Transport] Serial write timed out. Restore robot power, "
                      "then unplug/replug DAPLink USB and restart the program.")
                return False
            except serial.SerialException as e:
                print(f"[Transport] Send error: {e}")
                return False

    def send_command(self, cmd: int, data: bytes = b'') -> bool:
        """Alias for send()."""
        return self.send(cmd, data)

    # ======================== Receive =========================================

    def _recv_loop(self):
        """Background thread: read bytes, parse frames, dispatch callbacks."""
        last_debug = 0
        byte_count = 0
        while self._running:
            try:
                if self._ser and self._ser.in_waiting > 0:
                    chunk = self._ser.read(self._ser.in_waiting)
                    byte_count += len(chunk)
                    self._rx_buf.extend(chunk)
                    self._parse_frames()
                else:
                    time.sleep(0.001)  # 1ms idle

                # Debug: print raw stats every 2 seconds
                if self._debug:
                    now = time.time()
                    if now - last_debug >= 2.0:
                        print(f"  [DEBUG] rx_buf={len(self._rx_buf)}B, "
                              f"total={byte_count}B, frames={self.rx_frames}, "
                              f"crc_err={self.rx_crc_errors}")
                        if len(self._rx_buf) > 0:
                            hex_preview = self._rx_buf[:16].hex(' ')
                            print(f"  [DEBUG] buf head: {hex_preview}")
                        last_debug = now
            except (serial.SerialException, OSError) as e:
                print(f"[Transport] Recv error: {e}")
                time.sleep(0.1)

    def _parse_frames(self):
        """Scan rx_buf for complete frames and dispatch."""
        while len(self._rx_buf) >= 6:  # minimum frame: SYNC+CMD+LEN+SEQ+CRC=7 actually
            # Find sync byte
            try:
                sync_idx = self._rx_buf.index(PROTO_SYNC)
            except ValueError:
                self._rx_buf.clear()
                return

            # Discard bytes before sync
            if sync_idx > 0:
                self._rx_buf = self._rx_buf[sync_idx:]

            # Need at least: SYNC(1) + CMD(1) + LEN(1) + SEQ(1) + CRC(2) = 6 more bytes
            if len(self._rx_buf) < 7:
                return

            cmd  = self._rx_buf[1]
            flen = self._rx_buf[2]  # frame length

            if flen < 5 or flen > PROTO_MAX_DATA_LEN + 5:
                # Corrupted — discard sync byte and retry
                self._rx_buf = self._rx_buf[1:]
                continue

            total = 1 + flen  # SYNC + frame
            if len(self._rx_buf) < total:
                return  # incomplete frame

            frame_data = self._rx_buf[:total]

            # Verify CRC
            body = frame_data[1:1 + flen - 2]  # CMD..DATA (excludes CRC)
            rx_crc = struct.unpack_from('<H', frame_data, 1 + flen - 2)[0]
            calc_crc = crc16_ccitt(body)

            if rx_crc != calc_crc:
                self.rx_crc_errors += 1
                self._rx_buf = self._rx_buf[1:]  # skip sync, try next
                continue

            # Valid frame — dispatch
            seq  = frame_data[3]
            data = frame_data[4:4 + flen - 5]  # payload

            self.rx_frames += 1
            self._dispatch(cmd, seq, data)

            # Remove processed frame from buffer
            self._rx_buf = self._rx_buf[total:]

    def _dispatch(self, cmd: int, seq: int, data: bytes):
        """Route a received frame to the appropriate callback."""
        if cmd == TELEM_FULL:
            try:
                telem = TelemBatch.unpack(data)
                if self._on_telemetry:
                    self._on_telemetry(telem)
            except Exception as e:
                print(f"[Transport] Telemetry parse error: {e}")

        elif cmd == TELEM_ACK:
            try:
                ack = AckFrame.unpack(data)
                if self._on_ack:
                    self._on_ack(ack)
            except Exception:
                pass

        elif cmd == TELEM_PONG:
            try:
                uptime = struct.unpack('>I', data)[0] if len(data) >= 4 else 0
                if self._on_pong:
                    self._on_pong(uptime)
            except Exception:
                pass

    # ======================== High-Level Commands =============================

    def ping(self) -> bool:
        """Send a PING and return True if sent."""
        return self.send(CMD_PING)

    def emergency_stop(self) -> bool:
        """Send EMERGENCY_STOP — stops all motors immediately."""
        return self.send(CMD_EMERGENCY_STOP)

    def set_chassis_speed(self, rpm: list) -> bool:
        """Set 4 motor RPM targets. rpm = [tr, tl, bl, br] in RPM."""
        from .commands import encode_chassis_speed
        return self.send(CMD_CHASSIS_SPEED, encode_chassis_speed(rpm))

    def set_chassis_torque(self, torque: list) -> bool:
        """Set 4 motor raw torque values."""
        from .commands import encode_chassis_torque
        return self.send(CMD_CHASSIS_TORQUE, encode_chassis_torque(torque))

    def set_chassis_speed_pid(self, motor_id: int, kp: float, ki: float,
                               kd: float, ilim: float, olim: float) -> bool:
        """Set speed-loop PID gains for one motor."""
        from .commands import encode_chassis_pid_speed
        return self.send(CMD_CHASSIS_PID_SPEED,
                        encode_chassis_pid_speed(motor_id, kp, ki, kd, ilim, olim))

    def set_chassis_pos_pid(self, motor_id: int, kp: float, ki: float,
                             kd: float, ilim: float, olim: float) -> bool:
        """Set position-loop PID gains for one motor."""
        from .commands import encode_chassis_pid_pos
        return self.send(CMD_CHASSIS_PID_POS,
                        encode_chassis_pid_pos(motor_id, kp, ki, kd, ilim, olim))

    def reset_chassis_pid(self, motor_id: int) -> bool:
        """Reset PID integrators for one motor."""
        from .commands import encode_chassis_pid_reset
        return self.send(CMD_CHASSIS_PID_RESET, encode_chassis_pid_reset(motor_id))

    def set_servo_angle(self, servo_id: int, angle_deg: int) -> bool:
        """Set a single servo angle (0–180°)."""
        from .commands import encode_servo_angle
        return self.send(CMD_SERVO_ANGLE, encode_servo_angle(servo_id, angle_deg))

    def servo_home_all(self) -> bool:
        """Home all 4 servos."""
        return self.send(CMD_SERVO_HOME)

    def set_servo_all(self, angles: list) -> bool:
        """Set all 4 servo angles at once."""
        from .commands import encode_servo_angle_all
        return self.send(CMD_SERVO_ANGLE_ALL, encode_servo_angle_all(angles))

    def stepper_move(self, motor: int, direction: int, steps: int,
                     start_delay: int = 0, target_delay: int = 0,
                     accel_steps: int = 0) -> bool:
        """Launch a single-motor trapezoidal move (non-blocking)."""
        from .commands import encode_stepper_move
        return self.send(CMD_STEPPER_MOVE,
                        encode_stepper_move(motor, direction, steps,
                                           start_delay, target_delay, accel_steps))

    def stepper_stop(self, motor: int) -> bool:
        """Emergency-stop a stepper motor."""
        from .commands import encode_stepper_stop
        return self.send(CMD_STEPPER_STOP, encode_stepper_stop(motor))

    def stepper_set_params(self, start_delay: int, target_delay: int,
                           accel_steps: int) -> bool:
        """Set default trapezoidal parameters."""
        from .commands import encode_stepper_params
        return self.send(CMD_STEPPER_PARAMS,
                        encode_stepper_params(start_delay, target_delay, accel_steps))

    def stepper_move_dual(self, m1: int, steps1: int, dir1: int,
                          m2: int, steps2: int, dir2: int,
                          m2_offset: int = 0,
                          start_delay: int = 1000, target_delay: int = 100,
                          accel_steps: int = 400) -> bool:
        """Launch dual-motor overlapping move."""
        from .commands import encode_stepper_move_dual
        return self.send(CMD_STEPPER_MOVE_DUAL,
                        encode_stepper_move_dual(m1, steps1, dir1,
                                                m2, steps2, dir2,
                                                m2_offset,
                                                start_delay, target_delay,
                                                accel_steps))

    def stepper_move_dual2(self,
                           m_cont: int, steps_cont: int, dir_cont: int,
                           m_ph: int, steps_ph1: int, dir_ph1: int,
                           steps_ph2: int, dir_ph2: int,
                           ph2_offset: int,
                           start_delay: int = 1000, target_delay: int = 100,
                           accel_steps: int = 400) -> bool:
        """Launch dual-motor move with mid-move direction change."""
        from .commands import encode_stepper_move_dual2
        return self.send(CMD_STEPPER_MOVE_DUAL2,
                        encode_stepper_move_dual2(
                            m_cont, steps_cont, dir_cont,
                            m_ph, steps_ph1, dir_ph1,
                            steps_ph2, dir_ph2,
                            ph2_offset,
                            start_delay, target_delay, accel_steps))

    def stepper_move_dual3(
            self, m_lead: int, steps_lead1: int, dir_lead1: int,
            steps_lead2: int, dir_lead2: int,
            m_other: int, steps_other: int, dir_other: int,
            other_offset: int, lead2_offset: int,
            start_delay: int = 1000, target_delay: int = 100,
            accel_steps: int = 400) -> bool:
        """Launch a cross-triggered three-segment dual-motor move."""
        return self.send(
            CMD_STEPPER_MOVE_DUAL3,
            encode_stepper_move_dual3(
                m_lead, steps_lead1, dir_lead1,
                steps_lead2, dir_lead2,
                m_other, steps_other, dir_other,
                other_offset, lead2_offset,
                start_delay, target_delay, accel_steps))

    def stepper_set_position(self, motor: int, pos: int) -> bool:
        """Set stepper cumulative position counter."""
        from .commands import encode_stepper_set_pos
        return self.send(CMD_STEPPER_SET_POS, encode_stepper_set_pos(motor, pos))

    def set_telemetry_rate(self, rate_hz: int) -> bool:
        """Set telemetry streaming rate (0 = off, 1–200 Hz)."""
        from .commands import encode_set_telem_rate
        return self.send(CMD_SET_TELEM_RATE, encode_set_telem_rate(rate_hz))
