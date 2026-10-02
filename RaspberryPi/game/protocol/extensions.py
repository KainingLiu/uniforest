"""Opt-in additive wire interface. Legacy schema v3 remains frozen."""
import struct
import threading
import time

CMD_EXEC_CAPABILITIES = 0x52
CMD_EXEC_OPEN = 0x53
CMD_EXEC_COMMAND = 0x54
CMD_EXEC_CLOSE = 0x55
TELEM_EXEC_CAPABILITIES = 0x84
TELEM_EXEC_SESSION = 0x85
TELEM_EXEC_ACTION = 0x86
VERSION = 1
CAPABILITIES = 7
LEASE_MS = 200
LENGTHS = {0x01:(0,), 0x02:(0,), 0x10:(8,), 0x20:(2,3), 0x21:(0,),
           0x22:(4,), 0x23:(1,), 0x30:(6,), 0x31:(1,), 0x33:(22,),
           0x34:(27,), 0x36:(31,), 0x40:(2,), 0x50:(6,), 0x51:(0,)}


class ExecutionLink:
    """A failed owner can never fall back to raw legacy motor commands."""
    def __init__(self, transport):
        self.transport = transport
        self.condition = threading.Condition(threading.RLock())
        self.session = 0  # assigned monotonically by this MCU boot
        self.active = False
        self.failed = False
        self.error = ''
        self.waiting = None
        self.response = None
        self.pending = {}

    def fail(self, reason):
        with self.condition:
            self.failed = True
            self.active = False
            self.error = reason
            self.condition.notify_all()

    def _request(self, command, data, response, timeout):
        with self.transport._tx_lock, self.condition:
            seq = (self.transport._seq + 1) & 255
            self.waiting = (response, seq, command)
            self.response = None
            if not self.transport._send_raw(command, data):
                self.fail('extension request send failed')
        with self.condition:
            deadline = time.monotonic() + timeout
            while self.response is None and not self.failed:
                left = deadline-time.monotonic()
                if left <= 0:
                    self.fail('extension handshake timed out; matching additive firmware required')
                    break
                self.condition.wait(left)
            self.waiting = None
            if self.failed:
                raise RuntimeError(self.error)
            return self.response

    def open(self, timeout=.15):
        try:
            caps = self._request(CMD_EXEC_CAPABILITIES,b'',TELEM_EXEC_CAPABILITIES,timeout)
            if len(caps)!=12:
                raise RuntimeError('invalid firmware capability payload')
            version, flags, lease, offered = struct.unpack('>HIHI',caps)
            if (version!=VERSION or flags & CAPABILITIES != CAPABILITIES
                    or lease!=LEASE_MS or offered==0):
                raise RuntimeError('firmware execution capabilities do not match')
            self.session = offered
            reply = self._request(CMD_EXEC_OPEN,struct.pack('>IH',self.session,VERSION),
                                  TELEM_EXEC_SESSION,timeout)
            if reply!=struct.pack('>IB',self.session,1):
                raise RuntimeError('firmware did not grant the requested execution session')
            with self.condition:
                self.active = True
        except BaseException as exc:
            self.fail(str(exc))
            # Cleanup is scoped to our offered ID. A rejected/lost OPEN must
            # never send a raw stop into a different client's legacy run.
            if self.session:
                self.transport._send_raw(CMD_EXEC_CLOSE,struct.pack('>I',self.session))
            raise

    def encode(self, command, data, seq):
        with self.condition:
            if self.failed or not self.active:
                raise RuntimeError(self.error or 'execution session is closed')
            if command not in LENGTHS or len(data) not in LENGTHS[command]:
                raise ValueError('command is outside the match-session interface')
            self.pending[seq] = command
            return CMD_EXEC_COMMAND, struct.pack('>IB',self.session,command)+data

    def receive(self, command, seq, data):
        with self.condition:
            if self.waiting and (command,seq)==self.waiting[:2]:
                self.response = bytes(data)
                self.condition.notify_all()
                return True
            if command==0x81 and len(data)>=2:
                if self.waiting and seq==self.waiting[1] and data[0]==self.waiting[2]:
                    if data[1]: self.fail(f'firmware rejected extension handshake: {data[1]}')
                elif seq in self.pending and data[0] in (CMD_EXEC_COMMAND,self.pending[seq]):
                    self.pending.pop(seq,None)
                    if data[1]: self.fail(f'firmware rejected execution command: {data[1]}')
            return command in (TELEM_EXEC_CAPABILITIES,TELEM_EXEC_SESSION)

    def action_payload(self, data):
        with self.condition:
            if self.failed or not self.active or len(data)!=15:
                return None
            if struct.unpack_from('>I',data)[0]!=self.session:
                return None
            return data[4:]
