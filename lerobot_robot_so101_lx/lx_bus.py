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


class _BadReply(Exception):
    """Ответ мотора не пришёл целиком или испорчен; запрос будет повторён."""


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
                if method == "get_position":
                    return self._read_position_fast(motor_id)
                return getattr(ctrl, method)(motor_id, self.timeout)
            except (lx.TimeoutError, IndexError, _BadReply) as e:
                last_error = e
        raise LXBusError(
            f"Мотор '{motor}' (id={motor_id}) не ответил за {1 + self.num_retry} попыток."
        ) from last_error

    def _read_position_fast(self, motor_id: int) -> int:
        """Запросить позицию и прочитать ответ одним обращением к порту.

        Библиотека читает ответ пятью кусочками и перед каждым меняет таймаут порта,
        что в Windows заново настраивает порт. Позиция читается в каждом цикле
        управления, поэтому здесь ответ известной длины (8 байт) читается целиком.
        """
        ser = self._serial
        if ser.timeout != self.timeout:  # библиотека могла изменить таймаут
            ser.timeout = self.timeout
        length, command = 3, lx.SERVO_POS_READ
        checksum = 255 - ((motor_id + length + command) % 256)
        ser.write(bytes([0x55, 0x55, motor_id, length, command, checksum]))

        reply = ser.read(8)  # 55 55 id 05 1C младший старший контрольная_сумма
        if len(reply) != 8:
            raise _BadReply(f"пришло {len(reply)} байт из 8")
        header_ok = reply[0] == 0x55 and reply[1] == 0x55
        fields_ok = reply[2] == motor_id and reply[3] == 5 and reply[4] == command
        checksum_ok = reply[7] == 255 - (sum(reply[2:7]) % 256)
        if not (header_ok and fields_ok and checksum_ok):
            raise _BadReply(f"испорченный ответ: {list(reply)}")

        position = reply[5] + reply[6] * 256
        return position - 65536 if position > 32767 else position

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
