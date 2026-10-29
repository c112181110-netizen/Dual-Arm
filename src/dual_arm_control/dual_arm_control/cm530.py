"""CM530 v17/protocol 5. No ROS imports and no implicit motion/retry."""
import math
import threading
import time
from dataclasses import dataclass


class ProtocolError(RuntimeError):
    pass


class CommandError(RuntimeError):
    """A complete ERR reply; the transaction ended, but its motion failed."""


def parse_reply(command, line):
    cmd = command.split(',')
    fields = line.split(',')
    if fields[0] == 'ERR':
        raise CommandError(line)
    if cmd[0] in ('READ', 'GET_HOME'):
        tag = 'POS' if cmd[0] == 'READ' else 'HOME'
        if len(fields) != 6 or fields[:2] != [tag, cmd[1]]:
            raise ProtocolError(f'Mismatched reply: {command!r} -> {line!r}')
        if not all(s.isascii() and s.isdigit() and 0 <= int(s) <= 1023 for s in fields[2:]):
            raise ProtocolError(f'Invalid raw positions: {line}')
        return [int(s) for s in fields[2:]]
    expected = {'PING': ['PONG'], 'VERSION': ['VERSION', '5']}.get(cmd[0])
    if expected is None:
        expected = ['OK', *cmd[:2]]
        if cmd[0] in ('TORQUE', 'LED'):
            expected.append(cmd[2])
    if fields != expected:
        raise ProtocolError(f'Mismatched reply: {command!r} -> {line!r}')
    return None


@dataclass
class Calibration:
    zero: list
    direction: list
    lower: list
    upper: list
    raw_min: list
    raw_max: list
    ticks_per_rad: float = 1023.0 / math.radians(300.0)

    def __post_init__(self):
        arrays = [self.zero, self.direction, self.lower, self.upper, self.raw_min, self.raw_max]
        if any(len(a) != 4 for a in arrays) or any(not math.isfinite(float(v)) for a in arrays for v in a):
            raise ValueError('Four finite calibration values are required per arm')
        if any(d not in (-1, 1) for d in self.direction):
            raise ValueError('Joint directions must be -1 or +1')
        if not math.isfinite(self.ticks_per_rad) or self.ticks_per_rad <= 0:
            raise ValueError('Invalid ticks_per_rad')
        for i in range(4):
            if not self.lower[i] < self.upper[i] or not 0 <= self.raw_min[i] < self.raw_max[i] <= 1023:
                raise ValueError('Invalid mechanical limits')
            if not self.raw_min[i] <= self.zero[i] <= self.raw_max[i]:
                raise ValueError('Zero outside raw limits')

    def to_raw(self, positions):
        if len(positions) != 4:
            raise ValueError('Exactly four joint positions required')
        result = []
        for i, q in enumerate(positions):
            if not math.isfinite(q) or not self.lower[i] <= q <= self.upper[i]:
                raise ValueError(f'Joint {i} outside calibrated limits: {q}')
            raw = round(self.zero[i] + self.direction[i] * q * self.ticks_per_rad)
            if not self.raw_min[i] <= raw <= self.raw_max[i]:
                raise ValueError(f'Joint {i} raw target outside calibrated limits: {raw}')
            result.append(raw)
        return result

    def to_rad(self, raw):
        if len(raw) != 4 or any(not isinstance(x, int) or not 0 <= x <= 1023 for x in raw):
            raise ValueError('Invalid READ positions')
        return [(raw[i] - self.zero[i]) / (self.direction[i] * self.ticks_per_rad) for i in range(4)]


class SerialCM530:
    def __init__(self, port, timeout=2.0, startup_listen=0.3):
        import serial
        self.port = serial.Serial(port, 57600, timeout=0.02, write_timeout=timeout, exclusive=True)
        self.timeout = timeout
        self.lock = threading.RLock()
        self.healthy = True
        # Boot messages may have been emitted before opening. Drain only before
        # the handshake, never to hide an ambiguous outstanding transaction.
        deadline = time.monotonic() + startup_listen
        boot = bytearray()
        while time.monotonic() < deadline:
            boot.extend(self.port.read(max(1, self.port.in_waiting)))
        if b'ERR,INIT_FAILED' in boot:
            self.healthy = False
            raise ProtocolError('CM530 initialization failed; reset board after diagnosis')
        self.transact('VERSION')
        self.transact('PING')

    def transact(self, command):
        with self.lock:
            if not self.healthy:
                raise ProtocolError('Serial connection faulted; no automatic resynchronization or retry')
            try:
                stale = self.port.read(self.port.in_waiting)
                if stale.strip(b'\r\n'):
                    raise ProtocolError(f'Unexpected data before request: {stale!r}')
                self.port.write((command + '\n').encode('ascii'))
                deadline = time.monotonic() + self.timeout
                data = bytearray()
                while time.monotonic() < deadline:
                    byte = self.port.read(1)
                    if not byte:
                        continue
                    if byte in (b'\r', b'\n'):
                        if data:
                            return parse_reply(command, data.decode('ascii'))
                        continue
                    data.extend(byte)
                    if len(data) > 256:
                        raise ProtocolError('Oversized reply')
                raise ProtocolError(f'Command timeout; outcome unknown: {command}')
            except CommandError:
                raise
            except Exception:
                self.healthy = False
                raise

    def read(self, arm):
        return self.transact(f'READ,{arm}')

    def write(self, arm, raw):
        self.transact('AX,' + arm + ',' + ','.join(map(str, raw)))

    def enable(self, arm):
        # Holding the measured pose prevents an old target from being activated.
        with self.lock:
            self.transact(f'HOLD,{arm}')
            self.transact(f'TORQUE,{arm},1')

    def hold(self, arm):
        self.transact(f'HOLD,{arm}')

    def close(self):
        self.port.close()


class MockCM530:
    """Ideal raw-position actuator, not a motor dynamics simulation."""
    def __init__(self, initial):
        self.positions = {arm: list(raw) for arm, raw in initial.items()}
        self.healthy = True
        self.lock = threading.RLock()

    def read(self, arm):
        with self.lock:
            return list(self.positions[arm])

    def write(self, arm, raw):
        with self.lock:
            self.positions[arm] = list(raw)

    def enable(self, arm):
        pass

    def hold(self, arm):
        pass

    def close(self):
        pass
