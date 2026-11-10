"""Тесты LXBus без железа: вместо порта — имитация сервоприводов LX."""

import pytest
import serial

from lerobot_robot_so101_lx import LXBus, LXBusError

POS_READ, VIN_READ, TEMP_READ, MOVE_WRITE, LOAD_WRITE = 28, 27, 26, 1, 31


def packet(servo_id: int, command: int, *params: int) -> bytes:
    length = 3 + len(params)
    checksum = 255 - ((servo_id + length + command + sum(params)) % 256)
    return bytes([0x55, 0x55, servo_id, length, command, *params, checksum])


class FakeSerial:
    """Отвечает как набор сервоприводов LX с заданными позициями."""

    def __init__(self, positions: dict[int, int]):
        self.positions = dict(positions)
        self.torque = {servo_id: False for servo_id in positions}
        self.move_times: dict[int, int] = {}
        self.drop_next = 0  # сколько ближайших запросов оставить без ответа
        self.is_open = True
        self.timeout = None
        self._rx = b""

    def write(self, data: bytes) -> int:
        data = bytes(data)
        servo_id, command, params = data[2], data[4], data[5:-1]
        if servo_id not in self.positions:
            return len(data)
        if command == MOVE_WRITE:
            self.positions[servo_id] = params[0] + params[1] * 256
            self.move_times[servo_id] = params[2] + params[3] * 256
            self.torque[servo_id] = True
        elif command == LOAD_WRITE:
            self.torque[servo_id] = bool(params[0])
        elif command in (POS_READ, VIN_READ, TEMP_READ):
            if self.drop_next > 0:
                self.drop_next -= 1
                return len(data)
            if command == POS_READ:
                value = self.positions[servo_id] % 65536
                self._rx += packet(servo_id, command, value % 256, value // 256)
            elif command == VIN_READ:
                self._rx += packet(servo_id, command, 7400 % 256, 7400 // 256)
            else:
                self._rx += packet(servo_id, command, 31)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        data, self._rx = self._rx[:size], self._rx[size:]
        return data

    def reset_input_buffer(self) -> None:
        self._rx = b""

    def close(self) -> None:
        self.is_open = False


@pytest.fixture
def fake(monkeypatch):
    port = FakeSerial({1: 500, 2: 120, 3: -5})
    monkeypatch.setattr(serial, "Serial", lambda *args, **kwargs: port)
    return port


def make_bus(**kwargs) -> LXBus:
    return LXBus("COM_TEST", {"a": 1, "b": 2, "c": 3}, timeout=0.01, **kwargs)


def test_read_positions_including_negative(fake):
    bus = make_bus()
    bus.connect()
    assert bus.read_positions() == {"a": 500, "b": 120, "c": -5}
    assert bus.read_positions(["b"]) == {"b": 120}
    assert bus.read_voltage("a") == pytest.approx(7.4)
    assert bus.read_temperature("a") == 31


def test_write_rounds_clamps_and_returns_sent(fake):
    bus = make_bus()
    bus.connect()
    sent = bus.write_positions({"a": 250.6, "b": 1500, "c": -20}, time_ms=33)
    assert sent == {"a": 251, "b": 1000, "c": 0}
    assert fake.positions == {1: 251, 2: 1000, 3: 0}
    assert fake.move_times == {1: 33, 2: 33, 3: 33}


def test_torque_and_disconnect(fake):
    bus = make_bus()
    bus.connect()
    bus.enable_torque()
    assert all(fake.torque.values())
    bus.disable_torque("a")
    assert fake.torque == {1: False, 2: True, 3: True}
    bus.disconnect()
    assert not any(fake.torque.values())
    assert not bus.is_connected
    with pytest.raises(LXBusError):
        bus.read_positions()


def test_retry_recovers_from_lost_reply(fake):
    bus = make_bus(num_retry=2)
    bus.connect()
    fake.drop_next = 2
    assert bus.read_position("a") == 500
    fake.drop_next = 3
    with pytest.raises(LXBusError):
        bus.read_position("a")


def test_handshake_reports_missing_motor(fake):
    bus = LXBus("COM_TEST", {"a": 1, "ghost": 9}, timeout=0.01, num_retry=0)
    with pytest.raises(LXBusError, match="ghost"):
        bus.connect()
    assert not bus.is_connected


def test_rejects_bad_configuration():
    with pytest.raises(ValueError):
        LXBus("COM_TEST", {"a": 1, "b": 1})
    with pytest.raises(ValueError):
        LXBus("COM_TEST", {})
