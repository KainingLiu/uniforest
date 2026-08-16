#!/usr/bin/env python3
"""
VOFA+ Bridge — reads STM32 binary telemetry and serves it to VOFA+.

Usage:
    python tools/vofa_bridge.py COM3 --format justfloat  # JustFloat (TCP:5555)
    python tools/vofa_bridge.py COM3 --format firewater  # FireWater (TCP:5555)
    python tools/vofa_bridge.py COM3 --format console    # terminal output

Then in VOFA+:
    1. Add a data source: TCP Client → localhost:5555
    2. Protocol: JustFloat (if using --format justfloat)
    3. Set frame size to auto-detect

Channels (JustFloat order):
    Ch0:   uptime_s    系统运行时间 (s)
    Ch1:   yaw_deg     IMU偏航角 (°)
    Ch2:   yaw_rate    IMU偏航角速度 (°/s)
    Ch3:   motor_TR    TR电机转速 (RPM)
    Ch4:   motor_TL    TL电机转速 (RPM)
    Ch5:   motor_BL    BL电机转速 (RPM)
    Ch6:   motor_BR    BR电机转速 (RPM)
    Ch7:   temp_TR     TR电机温度 (°C)
    Ch8:   temp_TL     TL电机温度 (°C)
    Ch9:   temp_BL     BL电机温度 (°C)
    Ch10:  temp_BR     BR电机温度 (°C)
    Ch11:  steer_busy 步进电机忙标志
    Ch12:  steer_H     水平步进位置 (steps)
    Ch13:  steer_V     垂直步进位置 (steps)
    Ch14:  rc_ch1      RC遥控通道1
    Ch15:  rc_ch2      RC遥控通道2
    Ch16:  rc_ch3      RC遥控通道3
    Ch17:  rc_ch4      RC遥控通道4
"""

import sys
import os
import struct
import socket
import time
import threading
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial
from utils.crc16 import crc16_ccitt

PROTO_SYNC = 0xAA
TELEM_FULL = 0x80


def parse_telem(data: bytes) -> dict:
    """Parse the 80-byte telemetry batch from the binary protocol."""
    if len(data) < 80:
        return None

    off = 0
    motors = []
    for i in range(4):
        angle = struct.unpack_from('>H', data, off)[0]
        rpm = struct.unpack_from('>h', data, off + 2)[0]
        torque = struct.unpack_from('>h', data, off + 4)[0]
        temp = data[off + 6]
        motors.append({'angle': angle, 'rpm': rpm, 'torque': torque, 'temp': temp})
        off += 7

    yaw = struct.unpack_from('>f', data, off)[0]; off += 4
    yaw_rate = struct.unpack_from('>f', data, off)[0]; off += 4
    rc = [struct.unpack_from('>H', data, off + i * 2)[0] for i in range(6)]
    off += 12
    stepper_busy = data[off]; off += 4
    uptime = struct.unpack_from('>I', data, off)[0]; off += 4
    stepper_pos = [struct.unpack_from('>i', data, off + i * 4)[0] for i in range(2)]
    off += 8
    motor_pos = [struct.unpack_from('>i', data, off + i * 4)[0] for i in range(4)]

    return {
        'uptime': uptime / 1000.0,
        'yaw': yaw,
        'yaw_rate': yaw_rate,
        'motors': motors,
        'rc': rc,
        'stepper_busy': stepper_busy,
        'stepper_pos': stepper_pos,
        'motor_pos': motor_pos,
    }


def justfloat_encode(telem: dict) -> bytes:
    """Pack telemetry as JustFloat (raw little-endian floats)."""
    m = telem['motors']
    floats = [
        float(telem['uptime']),
        telem['yaw'],
        telem['yaw_rate'],
        float(m[0]['rpm']), float(m[1]['rpm']),
        float(m[2]['rpm']), float(m[3]['rpm']),
        float(m[0]['temp']), float(m[1]['temp']),
        float(m[2]['temp']), float(m[3]['temp']),
        float(telem['stepper_busy']),
        float(telem['stepper_pos'][0]),
        float(telem['stepper_pos'][1]),
        float(telem['rc'][0]), float(telem['rc'][1]),
        float(telem['rc'][2]), float(telem['rc'][3]),
    ]
    return struct.pack(f'<{len(floats)}f', *floats)


def firewater_encode(telem: dict) -> bytes:
    """Pack telemetry as VOFA+ FireWater (CSV with float strings)."""
    m = telem['motors']
    values = [
        telem['uptime'], telem['yaw'], telem['yaw_rate'],
        m[0]['rpm'], m[1]['rpm'], m[2]['rpm'], m[3]['rpm'],
        m[0]['temp'], m[1]['temp'], m[2]['temp'], m[3]['temp'],
        telem['stepper_busy'],
        telem['stepper_pos'][0], telem['stepper_pos'][1],
        telem['rc'][0], telem['rc'][1], telem['rc'][2], telem['rc'][3],
    ]
    return ','.join(f'{v:.3f}' for v in values).encode() + b'\n'


class ProtocolReader:
    """Read binary protocol frames from serial port."""

    def __init__(self, port: str, baudrate: int = 115200):
        self._ser = serial.Serial(port, baudrate, timeout=0.01)
        self._buf = bytearray()
        self._frame_count = 0
        self._crc_errors = 0

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def read_frame(self) -> bytes | None:
        """Read one complete telemetry frame (returns raw data bytes)."""
        try:
            if self._ser.in_waiting > 0:
                self._buf.extend(self._ser.read(self._ser.in_waiting))
        except Exception:
            return None

        # Search for TELEM_FULL frame
        while len(self._buf) >= 7:
            try:
                idx = self._buf.index(PROTO_SYNC)
            except ValueError:
                self._buf.clear()
                return None

            if idx > 0:
                self._buf = self._buf[idx:]

            if len(self._buf) < 7:
                return None

            cmd = self._buf[1]
            flen = self._buf[2]

            if cmd != TELEM_FULL or flen < 5:
                self._buf = self._buf[1:]
                continue

            total = 1 + flen
            if len(self._buf) < total:
                return None

            frame = self._buf[:total]
            body = frame[1:1 + flen - 2]
            rx_crc = struct.unpack_from('<H', frame, 1 + flen - 2)[0]
            calc_crc = crc16_ccitt(body)

            if rx_crc != calc_crc:
                self._crc_errors += 1
                self._buf = self._buf[1:]
                continue

            data = frame[4:4 + flen - 5]
            self._frame_count += 1
            self._buf = self._buf[total:]
            return data

        return None


def main():
    parser = argparse.ArgumentParser(
        description='VOFA+ Bridge — STM32 binary protocol to VOFA+')
    parser.add_argument('port', help='Serial port (e.g. COM3)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--format', choices=['justfloat', 'firewater', 'console'],
                        default='justfloat',
                        help='Output format (default: justfloat)')
    parser.add_argument('--tcp-port', type=int, default=5555,
                        help='TCP server port for VOFA+ (default: 5555)')
    parser.add_argument('--no-tcp', action='store_true',
                        help='Disable TCP server, output to stdout only')
    args = parser.parse_args()

    reader = ProtocolReader(args.port, args.baud)
    print(f'[Bridge] Connected to {args.port} @ {args.baud}')
    print(f'[Bridge] Format: {args.format}')

    if not args.no_tcp and args.format != 'console':
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', args.tcp_port))
        server.listen(1)
        server.settimeout(1.0)
        print(f'[Bridge] TCP server on port {args.tcp_port} — waiting for VOFA+...')

        client = None
        try:
            client, addr = server.accept()
            print(f'[Bridge] VOFA+ connected from {addr}')
        except socket.timeout:
            print(f'[Bridge] No VOFA+ connection yet — TCP server still waiting')

        tcp_enabled = True
    else:
        client = None
        tcp_enabled = False
        server = None
        if args.format == 'console':
            print('[Bridge] Console mode — printing to stdout')

    print('[Bridge] Streaming... (Ctrl+C to stop)\n')

    try:
        last_print = time.time()
        frame_count = 0

        while True:
            data = reader.read_frame()
            if data is None:
                # Check for new TCP connections
                if tcp_enabled and client is None and server:
                    try:
                        client, addr = server.accept()
                        print(f'[Bridge] VOFA+ connected from {addr}')
                    except socket.timeout:
                        pass
                time.sleep(0.001)
                continue

            telem = parse_telem(data)
            if telem is None:
                continue

            frame_count += 1

            if args.format == 'justfloat':
                output = justfloat_encode(telem)
            elif args.format == 'firewater':
                output = firewater_encode(telem)
            else:  # console
                output = None

            # Send to TCP client
            if client and output:
                try:
                    client.sendall(output)
                except (ConnectionResetError, BrokenPipeError, OSError):
                    print('[Bridge] VOFA+ disconnected')
                    client.close()
                    client = None

            # Periodic status
            now = time.time()
            if now - last_print >= 1.0:
                fps = frame_count / (now - last_print)
                m = telem['motors']
                print(f'\r[Bridge] {fps:.0f} fps | Yaw={telem["yaw"]:.1f}deg | '
                      f'RPM=({m[0]["rpm"]:4d},{m[1]["rpm"]:4d},'
                      f'{m[2]["rpm"]:4d},{m[3]["rpm"]:4d}) '
                      f'| CRC err={reader._crc_errors}  ',
                      end='', flush=True)
                frame_count = 0
                last_print = now

    except KeyboardInterrupt:
        print('\n[Bridge] Stopped')
    finally:
        if client:
            client.close()
        if server:
            server.close()
        reader.close()


if __name__ == '__main__':
    main()
