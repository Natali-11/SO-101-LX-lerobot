"""Конфигурация робота SO-101 на сервоприводах LewanSoul LX."""

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig


def _default_motor_ids() -> dict[str, int]:
    # Полная SO-101: 6 моторов. Порядок ключей задаёт порядок значений в датасете,
    # он такой же, как у оригинальной SO-101.
    return {
        "shoulder_pan": 1,
        "shoulder_lift": 2,
        "elbow_flex": 3,
        "wrist_flex": 4,
        "wrist_roll": 5,
        "gripper": 6,
    }


@RobotConfig.register_subclass("so101_lx_follower")
@dataclass
class SO101LXFollowerConfig(RobotConfig):
    # Порт платы LewanSoul, в Windows вида "COM3".
    port: str

    # Имя сустава -> ID сервопривода на шине.
    motor_ids: dict[str, int] = field(default_factory=_default_motor_ids)

    # Отпустить моторы при отключении. Рука при этом упадёт под своим весом — придерживайте.
    disable_torque_on_disconnect: bool = True

    # Ограничение шага цели относительно текущего положения (в градусах; для захвата — в %).
    # Число — одно на все суставы, словарь — своё на каждый. Требует лишнего чтения позиций.
    max_relative_target: float | dict[str, float] | None = None

    # Время, за которое мотор отрабатывает одну команду, мс. Чуть больше периода цикла
    # (33 мс при 30 Гц), чтобы движение было плавным.
    move_time_ms: int = 40

    # Предел скорости суставов, градусов в секунду. Защищает от рывка, когда цель далеко
    # от текущего положения (например, в первый момент телеуправления). None — без предела.
    max_speed_deg_s: float | None = 180.0

    # Защита захвата от перегрева. У LX нельзя ограничить усилие, поэтому, если захват
    # упёрся в предмет (цель далеко, а губки не двигаются дольше gripper_stall_time_s),
    # цель подтягивается к фактическому положению: мотор продолжает сжимать, но слабее.
    # Защита снимается, как только захват открывают. None — защита выключена.
    gripper_stall_time_s: float | None = 0.5
    # Отставание от цели, начиная с которого захват считается упёршимся, шаги (≈7°).
    gripper_stall_steps: int = 30
    # Насколько дальше фактического положения остаётся цель при сжатии, шаги (≈3,6°).
    gripper_squeeze_steps: int = 15

    # Ожидание ответа мотора (с) и число повторных попыток чтения.
    read_timeout_s: float = 0.05
    num_read_retries: int = 2

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
