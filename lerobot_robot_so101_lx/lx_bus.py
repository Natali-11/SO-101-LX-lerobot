"""LXBus — тонкая обёртка над `lewansoul_lx16a.ServoController`.

Работает в сырых единицах сервопривода LX: позиция 0–1000 (0–240°, шаг ≈ 0,24°).
Перевод в градусы и калибровка — задача класса робота, не шины.

Имена методов (`enable_torque`, `disable_torque`, `is_connected`, `connect`,
`disconnect`) совпадают с шинами LeRobot, чтобы код робота выглядел привычно.
"""

from __future__ import annotations

import logging

import serial

from . import lewansoul_lx16a as lx

logger = logging.getLogger(__name__)

POSITION_MIN = 0
POSITION_MAX = 1000
DEGREES_PER_STEP = 240.0 / 1000.0


class LXBusError(RuntimeError):
    """Ошибка обмена с сервоприводами LX."""


class LXBus:
    """Несколько сервоприводов LewanSoul LX на одном последовательном порту.

    Пример:
        bus = LXBus("COM3", {"shoulder_pan": 1, "gripper": 5})
        bus.connect()
        print(bus.read_positions())
        bus.disconnect()
    """

    def __init__(
        self,
        port: str,
        motors: dict[str, int],
        baudrate: int = 115200,
        timeout: float = 0.05,
        num_retry: int = 2,
    ):
        """
        port: имя порта, в Windows вида "COM3".
        motors: имя сустава -> ID сервопривода на шине.
        timeout: сколько секунд ждать ответа одного мотора за одну попытку.
        num_retry: число повторных попыток чтения после неудачной.
        """
        if not motors:
            raise ValueError("Список моторов пуст.")
        ids = list(motors.values())
        if len(set(ids)) != len(ids):
            raise ValueError(f"ID моторов повторяются: {motors}")
        for name, motor_id in motors.items():
            if not 0 <= motor_id <= 253:
                raise ValueError(f"Недопустимый ID {motor_id} у мотора '{name}' (нужно 0–253).")

        self.port = port
        self.motors = dict(motors)
        self.baudrate = baudrate
        self.timeout = timeout
        self.num_retry = num_retry
        self._serial: serial.Serial | None = None
        self._ctrl: lx.ServoController | None = None

    # ---------------------------------------------------------------- связь

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self, handshake: bool = True) -> None:
        """Открыть порт. При handshake=True проверить, что все моторы отвечают."""
        if self.is_connected:
            raise LXBusError(f"Порт {self.port} уже открыт.")
        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout, write_timeout=1)
        self._ctrl = lx.ServoController(self._serial, timeout=self.timeout)

        if handshake:
            missing = [name for name in self.motors if not self.ping(name)]
            if missing:
                self._close()
                described = ", ".join(f"{name} (id={self.motors[name]})" for name in missing)
                raise LXBusError(
                    f"Не отвечают моторы: {described}. Проверьте питание 6–8,4 В, кабели и ID."
                )

    def disconnect(self, disable_torque: bool = True) -> None:
        """Закрыть порт; по умолчанию перед этим отпустить моторы."""
        if not self.is_connected:
            raise LXBusError("Шина не подключена.")
        try:
            if disable_torque:
                self.disable_torque()
        finally:
            self._close()

    def _close(self) -> None:
        if self._serial is not None:
            self._serial.close()
        self._serial = None
        self._ctrl = None

    def _controller(self) -> lx.ServoController:
        if not self.is_connected or self._ctrl is None:
            raise LXBusError("Шина не подключена. Сначала вызовите connect().")
        return self._ctrl

    def _names(self, motors: str | list[str] | None) -> list[str]:
        if motors is None:
            return list(self.motors)
        names = [motors] if isinstance(motors, str) else list(motors)
        for name in names:
            if name not in self.motors:
                raise KeyError(f"Неизвестный мотор '{name}'. Известные: {list(self.motors)}")
        return names

    # --------------------------------------------------------------- чтение

    def _query(self, method: str, motor: str) -> int:
        """Вызвать метод чтения контроллера с повторными попытками."""
        ctrl = self._controller()
        motor_id = self.motors[self._names(motor)[0]]
        last_error: Exception | None = None
        for _ in range(1 + self.num_retry):
            try:
                # Выбросить остатки опоздавших ответов, чтобы не принять старый ответ за новый.
                self._serial.reset_input_buffer()
                return getattr(ctrl, method)(motor_id, self.timeout)
            except (lx.TimeoutError, IndexError) as e:
                last_error = e
        raise LXBusError(
            f"Мотор '{motor}' (id={motor_id}) не ответил за {1 + self.num_retry} попыток."
        ) from last_error

    def ping(self, motor: str) -> bool:
        """True, если мотор отвечает на запрос позиции."""
        try:
            self._query("get_position", motor)
            return True
        except LXBusError:
            return False

    def read_position(self, motor: str) -> int:
        """Текущая позиция в шагах. Около упоров может выходить за 0–1000."""
        return self._query("get_position", motor)

    def read_positions(self, motors: str | list[str] | None = None) -> dict[str, int]:
        """Позиции нескольких моторов (по умолчанию всех) в шагах."""
        return {name: self._query("get_position", name) for name in self._names(motors)}

    def read_voltage(self, motor: str) -> float:
        """Напряжение питания мотора, В."""
        return self._query("get_voltage", motor) / 1000.0

    def read_temperature(self, motor: str) -> int:
        """Температура мотора, °C."""
        return self._query("get_temperature", motor)

    # --------------------------------------------------------------- запись

    def write_position(self, motor: str, position: float, time_ms: int = 0) -> int:
        """Отправить мотор в позицию (шаги) за time_ms миллисекунд. Включает удержание.

        Возвращает реально отправленное значение (округлённое и обрезанное до 0–1000).
        """
        ctrl = self._controller()
        name = self._names(motor)[0]
        target = int(round(position))
        target = max(POSITION_MIN, min(POSITION_MAX, target))
        ctrl.move(self.motors[name], target, int(time_ms))
        return target

    def write_positions(self, positions: dict[str, float], time_ms: int = 0) -> dict[str, int]:
        """Отправить несколько моторов в позиции. Возвращает реально отправленные значения."""
        return {name: self.write_position(name, pos, time_ms) for name, pos in positions.items()}

    def enable_torque(self, motors: str | list[str] | None = None) -> None:
        """Включить удержание (мотор сопротивляется повороту рукой)."""
        ctrl = self._controller()
        for name in self._names(motors):
            ctrl.motor_on(self.motors[name])

    def disable_torque(self, motors: str | list[str] | None = None) -> None:
        """Выключить удержание (мотор можно свободно поворачивать рукой)."""
        ctrl = self._controller()
        for name in self._names(motors):
            ctrl.motor_off(self.motors[name])
