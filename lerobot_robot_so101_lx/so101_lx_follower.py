"""Робот SO-101 на сервоприводах LewanSoul LX для LeRobot.

Единицы, как у оригинального SO-101 follower:
  * суставы руки — градусы, 0° в середине откалиброванного диапазона;
  * захват — проценты, 0 на одном краю диапазона, 100 на другом.

Калибровка хранится на компьютере в штатном файле LeRobot (в моторы ничего не пишется):
range_min / range_max — края диапазона в шагах мотора (0–1000), drive_mode = 1
разворачивает направление сустава, homing_offset не используется (всегда 0).
"""

import logging
import time
from functools import cached_property

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.motors import MotorCalibration
from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.utils import enter_pressed

from .config_so101_lx_follower import SO101LXFollowerConfig
from .lx_bus import DEGREES_PER_STEP, POSITION_MAX, POSITION_MIN, LXBus

logger = logging.getLogger(__name__)

GRIPPER = "gripper"
# Диапазон уже этого числа шагов (≈5°) считаем ошибкой калибровки: сустав не двигали.
MIN_RANGE_STEPS = 20


class SO101LXFollower(Robot):
    config_class = SO101LXFollowerConfig
    name = "so101_lx_follower"

    def __init__(self, config: SO101LXFollowerConfig):
        super().__init__(config)
        self.config = config
        self.bus = LXBus(
            port=config.port,
            motors=config.motor_ids,
            timeout=config.read_timeout_s,
            num_retry=config.num_read_retries,
        )
        self.cameras = make_cameras_from_configs(config.cameras)
        # Последние отправленные цели в шагах — для ограничения скорости.
        self._last_goal_raw: dict[str, int] = {}

    # ------------------------------------------------------------ описание

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {key: (cam.height, cam.width, 3) for key, cam in self.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # --------------------------------------------------------- подключение

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Открыть порт и камеры. Удержание моторов включится только с первой командой."""
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info("Файл калибровки не найден или не подходит к моторам — запускаю калибровку.")
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    def configure(self) -> None:
        # Моторам LX перед работой ничего записывать не нужно.
        self._last_goal_raw = {}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")

    # ---------------------------------------------------------- калибровка

    @property
    def is_calibrated(self) -> bool:
        """Есть калибровка для каждого мотора, ID совпадают, диапазоны осмысленные."""
        for motor, motor_id in self.bus.motors.items():
            cal = self.calibration.get(motor)
            if cal is None or cal.id != motor_id or cal.range_max - cal.range_min < MIN_RANGE_STEPS:
                return False
        return True

    def calibrate(self) -> None:
        if self.calibration and self.is_calibrated:
            answer = input(
                f"Найден файл калибровки для id '{self.id}'. ENTER — оставить его, "
                "'c' и ENTER — калибровать заново: "
            )
            if answer.strip().lower() != "c":
                return

        print(f"\nКалибровка {self}")
        input("Придержите руку и нажмите ENTER — моторы будут отпущены...")
        self.bus.disable_torque()

        motors = list(self.bus.motors)
        print(
            "\nПроведите КАЖДЫЙ сустав и захват от упора до упора (можно по очереди).\n"
            "Нажмите ENTER, когда закончите.\n"
        )
        start = self.bus.read_positions()
        mins, maxes = dict(start), dict(start)
        while True:
            positions = self.bus.read_positions()
            for motor, pos in positions.items():
                mins[motor] = min(mins[motor], pos)
                maxes[motor] = max(maxes[motor], pos)
            # Одна обновляемая строка: найденный диапазон каждого сустава в шагах.
            print("  ".join(f"{m}:{mins[m]}-{maxes[m]}" for m in motors) + "   ", end="\r", flush=True)
            if enter_pressed():
                break
        print()

        not_moved = []
        self.calibration = {}
        for motor in motors:
            range_min = max(POSITION_MIN, mins[motor])
            range_max = min(POSITION_MAX, maxes[motor])
            if range_max - range_min < MIN_RANGE_STEPS:
                not_moved.append(motor)
            self.calibration[motor] = MotorCalibration(
                id=self.bus.motors[motor],
                drive_mode=0,
                homing_offset=0,
                range_min=range_min,
                range_max=range_max,
            )
        if not_moved:
            self.calibration = {}
            raise RuntimeError(
                f"Суставы почти не двигались: {not_moved}. Калибровка не сохранена, запустите её ещё раз."
            )

        self._save_calibration()
        print(f"\n{'сустав':<15}{'мин':>6}{'макс':>6}{'ход, °':>9}")
        for motor, cal in self.calibration.items():
            span = (cal.range_max - cal.range_min) * DEGREES_PER_STEP
            print(f"{motor:<15}{cal.range_min:>6}{cal.range_max:>6}{span:>9.0f}")
        print("Калибровка сохранена в", self.calibration_fpath)

    # ------------------------------------------------- перевод шаги <-> единицы

    def _normalize(self, motor: str, raw: int) -> float:
        cal = self.calibration[motor]
        if motor == GRIPPER:
            bounded = min(cal.range_max, max(cal.range_min, raw))
            value = (bounded - cal.range_min) / (cal.range_max - cal.range_min) * 100
            return 100 - value if cal.drive_mode else value
        mid = (cal.range_min + cal.range_max) / 2
        value = (raw - mid) * DEGREES_PER_STEP
        return -value if cal.drive_mode else value

    def _unnormalize(self, motor: str, value: float) -> int:
        """Перевести в шаги и обрезать до откалиброванного диапазона."""
        cal = self.calibration[motor]
        if motor == GRIPPER:
            value = 100 - value if cal.drive_mode else value
            value = min(100.0, max(0.0, value))
            raw = value / 100 * (cal.range_max - cal.range_min) + cal.range_min
        else:
            value = -value if cal.drive_mode else value
            mid = (cal.range_min + cal.range_max) / 2
            raw = value / DEGREES_PER_STEP + mid
        return int(min(cal.range_max, max(cal.range_min, round(raw))))

    def _require_calibration(self) -> None:
        if not self.is_calibrated:
            raise RuntimeError(f"{self} не откалиброван. Запустите lerobot-calibrate.")

    def _read_normalized(self) -> dict[str, float]:
        raw = self.bus.read_positions()
        return {motor: self._normalize(motor, pos) for motor, pos in raw.items()}

    # ------------------------------------------------------ работа в цикле

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        self._require_calibration()
        start = time.perf_counter()
        obs_dict: RobotObservation = {f"{motor}.pos": val for motor, val in self._read_normalized().items()}
        logger.debug(f"{self} read state: {(time.perf_counter() - start) * 1e3:.1f}ms")

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            logger.debug(f"{self} read {cam_key}: {(time.perf_counter() - start) * 1e3:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Отправить суставам целевые положения (градусы; захват — проценты).

        Возвращает реально отправленное действие: цели обрезаются до откалиброванного
        диапазона и, если задан `max_relative_target`, до допустимого шага.
        """
        self._require_calibration()
        goal_pos = {key.removesuffix(".pos"): float(val) for key, val in action.items() if key.endswith(".pos")}
        unknown = [motor for motor in goal_pos if motor not in self.bus.motors]
        if unknown:
            raise KeyError(f"Неизвестные суставы в действии: {unknown}. Известные: {list(self.bus.motors)}")

        if self.config.max_relative_target is not None:
            present_pos = self._read_normalized()
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        goal_raw = {motor: self._unnormalize(motor, val) for motor, val in goal_pos.items()}

        # Для расчёта времени движения нужна точка отсчёта: до первой команды это текущее положение.
        missing = [motor for motor in goal_raw if motor not in self._last_goal_raw]
        if missing and self.config.max_speed_deg_s is not None:
            self._last_goal_raw.update(self.bus.read_positions(missing))

        for motor, raw in goal_raw.items():
            self.bus.write_position(motor, raw, self._move_time_ms(motor, raw))
            self._last_goal_raw[motor] = raw

        return {f"{motor}.pos": self._normalize(motor, raw) for motor, raw in goal_raw.items()}

    def _move_time_ms(self, motor: str, goal_raw: int) -> int:
        """Время движения: не меньше move_time_ms и не быстрее max_speed_deg_s."""
        move_time = self.config.move_time_ms
        if self.config.max_speed_deg_s is not None:
            distance_deg = abs(goal_raw - self._last_goal_raw[motor]) * DEGREES_PER_STEP
            move_time = max(move_time, distance_deg / self.config.max_speed_deg_s * 1000)
        return int(round(move_time))
