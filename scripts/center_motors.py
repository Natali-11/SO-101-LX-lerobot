"""Подготовка моторов к сборке руки: назначение ID и выставление в середину шкалы.

Примеры (из корня репозитория, в окружении conda `lerobot`):

    Назначить ID новому мотору (на шине должен быть ОДИН мотор):
        python scripts\\center_motors.py --port COM4 --set-id 6

    Выставить моторы в середину шкалы (позиция 500) и держать там:
        python scripts\\center_motors.py --port COM4 --ids 1 2 3 4 5 6

Во втором режиме моторы держат позицию, пока вы надеваете качалки: так вал
не провернётся. После ENTER удержание выключается.
"""

import argparse
import sys
import time

import serial

from lerobot_robot_so101_lx import LXBus, LXBusError
from lerobot_robot_so101_lx import lewansoul_lx16a as lx

MIDDLE = 500


def set_id(port: str, new_id: int) -> int:
    if not 1 <= new_id <= 253:
        print("ID должен быть от 1 до 253.")
        return 1
    ser = serial.Serial(port, 115200, timeout=0.1)
    ctrl = lx.ServoController(ser, timeout=0.1)
    try:
        try:
            # Общий запрос «кто здесь»: на него отвечает любой мотор, поэтому на шине
            # должен быть ровно один, иначе ответы перемешаются.
            old_id = ctrl.get_servo_id()
        except lx.TimeoutError:
            print("Мотор не ответил. Проверьте питание и кабель.")
            return 1
        print(f"Найден мотор с ID {old_id}.")
        if old_id == new_id:
            print("Этот ID уже назначен, менять нечего.")
            return 0
        input(f"Убедитесь, что к плате подключён ТОЛЬКО этот мотор, и нажмите ENTER — ему будет назначен ID {new_id}...")
        ctrl.set_servo_id(old_id, new_id)
        time.sleep(0.2)
        try:
            ctrl.get_position(new_id, 0.1)
        except lx.TimeoutError:
            print(f"Мотор не отвечает по новому ID {new_id}. Запустите команду ещё раз.")
            return 1
        print(f"Готово: мотору назначен ID {new_id}. Подпишите его.")
        return 0
    finally:
        ser.close()


def center(port: str, ids: list[int], position: int, time_ms: int) -> int:
    bus = LXBus(port, {f"id{i}": i for i in ids})
    bus.connect(handshake=False)
    try:
        missing = [name for name in bus.motors if not bus.ping(name)]
        if missing:
            print("Не отвечают моторы с ID:", ", ".join(str(bus.motors[n]) for n in missing))
            print("Проверьте питание, кабели и ID (python scripts\\center_motors.py --port ... --set-id ...).")
            return 1

        before = bus.read_positions()
        print("Сейчас:  ", "  ".join(f"ID {bus.motors[n]}: {p}" for n, p in before.items()))
        input(f"Валы моторов повернутся в позицию {position}. Снимите качалки или освободите суставы и нажмите ENTER...")

        bus.write_positions({name: position for name in bus.motors}, time_ms=time_ms)
        time.sleep(time_ms / 1000 + 0.5)
        after = bus.read_positions()
        print("Стало:   ", "  ".join(f"ID {bus.motors[n]}: {p}" for n, p in after.items()))
        off = [bus.motors[n] for n, p in after.items() if abs(p - position) > 10]
        if off:
            print(f"Не дошли до {position}: ID {off}. Возможно, вал чем-то зажат — освободите его и повторите.")

        print("\nМоторы держат позицию. Поставьте каждый сустав в середину его хода и наденьте качалки.")
        input("Когда закончите, придержите руку и нажмите ENTER — удержание будет выключено...")
        bus.disable_torque()
        return 1 if off else 0
    except LXBusError as e:
        print(f"Ошибка шины: {e}")
        return 1
    finally:
        bus.disconnect(disable_torque=False)


def main() -> int:
    parser = argparse.ArgumentParser(description="Назначение ID и выставление моторов LX в середину шкалы")
    parser.add_argument("--port", required=True, help="порт, например COM4")
    parser.add_argument("--set-id", type=int, help="назначить этот ID единственному мотору на шине")
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6], help="какие моторы выставить")
    parser.add_argument("--position", type=int, default=MIDDLE, help="позиция, по умолчанию 500 — середина")
    parser.add_argument("--time-ms", type=int, default=2000, help="время движения, мс")
    args = parser.parse_args()

    if args.set_id is not None:
        return set_id(args.port, args.set_id)
    if not 0 <= args.position <= 1000:
        print("Позиция должна быть от 0 до 1000.")
        return 1
    return center(args.port, args.ids, args.position, args.time_ms)


if __name__ == "__main__":
    sys.exit(main())
