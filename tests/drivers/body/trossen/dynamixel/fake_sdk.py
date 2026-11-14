"""Fake ``dynamixel_sdk``: handlers that answer like WidowX 250S servos on a U2D2.

Models the control-table reads the driver makes (present current / velocity /
position via GroupSyncRead, temperature / voltage / torque / hardware-error via
single reads) and records every write (``Bus.writes``). A goal-position write
moves that servo's present position straight to the goal.
"""

from __future__ import annotations

import sys
import types

COMM_SUCCESS = 0

# Present positions (ticks) measured on the real follower arm, servo id -> tick.
START_POS = {1: 2049, 2: 791, 4: 3078, 6: 2011, 7: 2645, 8: 2108, 9: 2936}

ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116


def _u(value: int, length: int) -> int:
    return value + (1 << (8 * length)) if value < 0 else value


class Bus:
    """The servos' state (ticks, torque, mode) and every write they received."""

    def __init__(self):
        self.pos = dict(START_POS)
        self.current = {sid: 0 for sid in START_POS}
        self.velocity = {sid: 0 for sid in START_POS}
        self.torque = {sid: 0 for sid in START_POS}
        self.mode = {sid: 3 for sid in START_POS}
        self.mode[9] = 16                    # gripper ships in PWM mode
        self.temp = {sid: 34 for sid in START_POS}
        self.voltage = {sid: 121 for sid in START_POS}   # 0.1 V units
        self.hw_error = {sid: 0 for sid in START_POS}
        self.present = True                  # whether servos answer a ping
        self.writes = []                     # (sid, addr, value)
        self.opened = 0
        self.closed = 0
        self.ports = []


def install(monkeypatch) -> Bus:
    bus = Bus()

    class PortHandler:
        def __init__(self, port):
            bus.ports.append(port)

        def openPort(self):
            bus.opened += 1
            return True

        def setBaudRate(self, baud):
            return True

        def closePort(self):
            bus.closed += 1

    class PacketHandler:
        def __init__(self, protocol):
            self.protocol = protocol

        def ping(self, ph, sid):
            if bus.present and sid in bus.pos:
                return (1020, COMM_SUCCESS, 0)
            return (0, -1000, 0)

        def read1ByteTxRx(self, ph, sid, addr):
            table = {ADDR_TORQUE_ENABLE: bus.torque, ADDR_OPERATING_MODE: bus.mode,
                     146: bus.temp, 70: bus.hw_error}
            return (table[addr][sid], COMM_SUCCESS, 0)

        def read2ByteTxRx(self, ph, sid, addr):
            if addr == 144:
                return (bus.voltage[sid], COMM_SUCCESS, 0)
            return (0, COMM_SUCCESS, 0)

        def read4ByteTxRx(self, ph, sid, addr):
            if addr == ADDR_PRESENT_POSITION:
                return (_u(bus.pos[sid], 4), COMM_SUCCESS, 0)
            return (0, COMM_SUCCESS, 0)

        def write1ByteTxRx(self, ph, sid, addr, value):
            bus.writes.append((sid, addr, value))
            if addr == ADDR_TORQUE_ENABLE:
                bus.torque[sid] = value
            elif addr == ADDR_OPERATING_MODE:
                bus.mode[sid] = value
            return (COMM_SUCCESS, 0)

        def write4ByteTxRx(self, ph, sid, addr, value):
            bus.writes.append((sid, addr, value))
            if addr == ADDR_GOAL_POSITION:
                bus.pos[sid] = value if value < (1 << 31) else value - (1 << 32)
            return (COMM_SUCCESS, 0)

    class GroupSyncRead:
        def __init__(self, ph, pk, start_addr, length):
            self.start = start_addr
            self.ids = []

        def addParam(self, sid):
            self.ids.append(sid)
            return True

        def txRxPacket(self):
            return COMM_SUCCESS

        def getData(self, sid, addr, length):
            if addr == ADDR_PRESENT_CURRENT:
                return _u(bus.current[sid], 2)
            if addr == ADDR_PRESENT_VELOCITY:
                return _u(bus.velocity[sid], 4)
            if addr == ADDR_PRESENT_POSITION:
                return _u(bus.pos[sid], 4)
            raise AssertionError(f"unexpected getData addr {addr}")

    mod = types.ModuleType("dynamixel_sdk")
    mod.PortHandler = PortHandler
    mod.PacketHandler = PacketHandler
    mod.GroupSyncRead = GroupSyncRead
    mod.COMM_SUCCESS = COMM_SUCCESS
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", mod)
    return bus


def goal_writes(bus: Bus):
    """Goal-position writes only, as (sid, tick)."""
    return [(sid, val) for sid, addr, val in bus.writes if addr == ADDR_GOAL_POSITION]
