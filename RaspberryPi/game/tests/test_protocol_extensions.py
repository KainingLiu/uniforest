"""Wire compatibility, opt-in negotiation, and fail-closed client ownership."""
import re
import json
import struct
from pathlib import Path
import unittest
from unittest.mock import patch

from protocol import extensions as ex
from protocol.commands import ActionStatus, TELEM_ACTION
from protocol.transport import Transport
from utils.crc16 import crc16_ccitt


class WireBoard:
    def __init__(self,transport,mode='ok'):
        self.transport=transport;self.mode=mode;self.is_open=True;self.frames=[]
    def write(self,frame):
        self.frames.append(frame)
        assert frame[0]==0xaa and len(frame)==frame[2]+1
        assert crc16_ccitt(frame[1:-2])==struct.unpack('<H',frame[-2:])[0]
        cmd,seq,data=frame[1],frame[3],frame[4:-2]
        if cmd==ex.CMD_EXEC_CAPABILITIES:
            if self.mode=='old': self.transport._dispatch(0x81,seq,bytes([cmd,4,0]))
            else:
                response=struct.pack('>HIHI',1,7 if self.mode!='wrong_caps' else 0,200,17)
                self.transport._dispatch(ex.TELEM_EXEC_CAPABILITIES,seq,response)
        elif cmd==ex.CMD_EXEC_OPEN:
            if self.mode=='busy': self.transport._dispatch(0x81,seq,bytes([cmd,2,0]))
            elif self.mode!='lost_open':
                self.transport._dispatch(ex.TELEM_EXEC_SESSION,seq,data[:4]+b'\1')
        return len(frame)
    def close(self): self.is_open=False


class ProtocolExtensionTests(unittest.TestCase):
    def transport(self,mode='ok'):
        t=Transport();t._ser=WireBoard(t,mode);return t

    def test_plain_transport_preserves_exact_legacy_frames(self):
        t=self.transport()
        t.ping(); t.start_action(0x12345678,2); t.set_chassis_speed([10,-20,30,-40])
        self.assertEqual([f[1] for f in t._ser.frames],[1,0x50,0x10])
        self.assertEqual(t._ser.frames[1][4:-2],bytes.fromhex('123456780200'))
        self.assertFalse(t.execution_active)

    def test_negotiation_and_all_match_commands_are_wrapped(self):
        t=self.transport();t.open_execution_session()
        t.ping();t.query_action_status();t.start_action(0x12345678,3)
        t.set_chassis_speed([10,-20,30,-40]);t.set_servo_angle(1,90)
        self.assertEqual([f[1] for f in t._ser.frames[:2]],[0x52,0x53])
        for f in t._ser.frames[2:]:
            self.assertEqual(f[1],0x54)
            self.assertEqual(struct.unpack('>I',f[4:8])[0],t._execution.session)
        self.assertEqual(t._ser.frames[4][8:-2],bytes.fromhex('50123456780300'))

    def test_legacy_status_cannot_release_enhanced_action(self):
        t=self.transport();t.open_execution_session()
        payload=struct.pack('>IBBBI',42,2,6,9,500)
        t._dispatch(TELEM_ACTION,1,payload)
        self.assertIsNone(t.get_action_status())
        t._dispatch(ex.TELEM_EXEC_ACTION,2,struct.pack('>I',t._execution.session^1)+payload)
        self.assertIsNone(t.get_action_status())
        t._dispatch(ex.TELEM_EXEC_ACTION,2,struct.pack('>I',t._execution.session)+payload)
        self.assertEqual(t.get_action_status()[0],ActionStatus(42,2,6,9,500))

    def test_stop_never_allows_silent_legacy_fallback(self):
        t=self.transport();t.open_execution_session();t.emergency_stop()
        self.assertEqual(t._ser.frames[-1][1],ex.CMD_EXEC_CLOSE)
        self.assertEqual(t._ser.frames[-1][4:-2],struct.pack('>I',t._execution.session))
        before=len(t._ser.frames)
        with self.assertRaises(RuntimeError):t.set_chassis_speed([100]*4)
        with self.assertRaises(RuntimeError):t.open_execution_session()
        self.assertEqual(len(t._ser.frames),before)

    def test_rejected_or_missing_handshake_fails_before_motion(self):
        for mode in ('old','wrong_caps','busy','lost_open'):
            with self.subTest(mode=mode):
                t=self.transport(mode)
                with self.assertRaises(RuntimeError):t.open_execution_session()
                self.assertFalse(t.execution_active)
                self.assertNotIn(0x54,[f[1] for f in t._ser.frames])
                self.assertNotIn(0x02,[f[1] for f in t._ser.frames])
                with self.assertRaises(RuntimeError):t.set_chassis_speed([0]*4)

    def test_wrong_reply_sequence_is_ignored(self):
        t=self.transport();link=ex.ExecutionLink(t)
        link.waiting=(ex.TELEM_EXEC_SESSION,25,ex.CMD_EXEC_OPEN)
        link.receive(ex.TELEM_EXEC_SESSION,24,struct.pack('>IB',link.session,1))
        self.assertIsNone(link.response)

    def test_envelope_rejection_latches_failure(self):
        t=self.transport();t.open_execution_session();t.ping()
        seq=t._seq
        t._dispatch(0x81,seq,bytes([0x54,1,0]))
        self.assertFalse(t.execution_active)
        with self.assertRaises(RuntimeError):t.query_action_status()

    def test_wire_send_failure_cannot_resume(self):
        t=self.transport();t.open_execution_session()
        with patch.object(t,'_send_raw',return_value=False):self.assertFalse(t.ping())
        with self.assertRaises(RuntimeError):t.ping()

    def test_persistent_tuning_and_position_reset_are_excluded(self):
        t=self.transport();t.open_execution_session()
        for cmd,data in ((0x12,bytes(21)),(0x32,bytes(6)),(0x35,bytes(5))):
            with self.assertRaises(ValueError):t.send(cmd,data)

    def test_firmware_extension_ids_match_client(self):
        from tests.project_paths import FIRMWARE as firmware
        if not firmware.is_dir():
            self.skipTest('A-board source checks run on the development checkout')
        source=(firmware/'Core/Inc/protocol.h').read_text(encoding='utf-8')
        ids={k:int(v,16) for k,v in re.findall(r'#define\s+((?:CMD|TELEM)_EXEC_\w+)\s+(0x[0-9A-Fa-f]+)',source)}
        self.assertEqual(len(ids),7)
        for name,value in ids.items():self.assertEqual(getattr(ex,name),value)

    def test_extension_schema_keeps_lengths_and_inner_allowlist_in_sync(self):
        doc=json.loads((Path(ex.__file__).parent/'extension_schema.json').read_text(encoding='utf-8'))
        self.assertEqual({int(k):tuple(v) for k,v in doc['inner_command_lengths'].items()},ex.LENGTHS)
        for group in ('commands','telemetry'):
            for name,item in doc[group].items():self.assertEqual(getattr(ex,name),item['id'])
        self.assertEqual(doc['telemetry']['TELEM_EXEC_CAPABILITIES']['data_length'],12)
        self.assertEqual(doc['telemetry']['TELEM_EXEC_ACTION']['data_length'],15)

    def test_partial_serial_write_invalidates_session(self):
        t=self.transport();t.open_execution_session()
        with patch.object(t._ser,'write',return_value=2):
            self.assertFalse(t.set_chassis_speed([100]*4))
        self.assertFalse(t.execution_active)
        with self.assertRaises(RuntimeError):t.ping()

    def test_full_lift_cannot_be_claimed_on_legacy_transport(self):
        from types import SimpleNamespace
        from control.actions import ActionSession
        t=self.transport()
        actions=SimpleNamespace(_t=t,pickup_full_lift_validated=True,_check_cancelled=lambda:None)
        with self.assertRaisesRegex(RuntimeError,'negotiated'):
            ActionSession(actions,3)._start()
        self.assertEqual(t._ser.frames,[])
