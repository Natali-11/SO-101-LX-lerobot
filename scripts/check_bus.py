"""Проверка шины LX и замер частоты цикла.

Примеры (из корня репозитория, в окружении conda `lerobot`):
    python scripts\\check_bus.py --port COM3
    python scripts\\check_bus.py --port COM3 --ids 1 2 3 4 5 --write

Без --write скрипт только читает: моторы не двигаются и не включают удержание.
С --write каждый цикл отправляет моторам их же текущую позицию: рука остаётся
на месте, но моторы включают удержание. По окончании удержание выключается —
придержите руку, чтобы она не упала.
"""

import argparse
import statistics
import sys
import time

from lerobot_robot_so101_lx import LXBus, LXBusError

JOINTS_5 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "gripper"]
JOINTS_6 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def report(title: str, durations: list[float], target_hz: float) -> None:
    ms = sorted(d * 1000 for d in durations)
    mean = statistics.fmean(ms)
    p95 = ms[int(0.95 * (len(ms) - 1))]
    hz = 1000 / mean
    verdict = "хватает" if hz >= target_hz else "НЕ хватает"
    print(f"{title}: среднее {mean:.1f} мс, 95% циклов до {p95:.1f} мс, худший {ms[-1]:.1f} мс")
    print(f"  -> {hz:.0f} Гц, для цели {target_hz:.0f} Гц {verdict}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Проверка шины LX и замер частоты цикла")
    parser.add_argument("--port", required=True, help="порт, например COM3")
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5], help="ID моторов по порядку суставов")
    parser.add_argument("--cycles", type=int, default=200, help="число циклов в каждом замере")
    parser.add_argument("--target-hz", type=float, default=30.0, help="нужная частота цикла")
    parser.add_argument("--write", action="store_true", help="замерить ещё и цикл чтение + запись")
    args = parser.parse_args()

    if len(args.ids) == 5:
        names = JOINTS_5
    elif len(args.ids) == 6:
        names = JOINTS_6
    else:
        names = [f"motor_{i}" for i in args.ids]
    motors = dict(zip(names, args.ids))

    bus = LXBus(args.port, motors)
    bus.connect(handshake=False)
    try:
        print(f"Порт {args.port}, моторы: {motors}\n")
        missing = []
        for name in motors:
            if not bus.ping(name):
                print(f"  {name:14s} id={motors[name]:<3d} НЕ ОТВЕЧАЕТ")
                missing.append(name)
                continue
            pos = bus.read_position(name)
            volts = bus.read_voltage(name)
            temp = bus.read_temperature(name)
            print(f"  {name:14s} id={motors[name]:<3d} позиция {pos:5d}   {volts:.1f} В   {temp} °C")
        if missing:
            print("\nНе все моторы отвечают — замер пропущен. Проверьте питание, кабели и ID.")
            return 1

        print(f"\nЗамер 1: чтение позиций {len(motors)} моторов, {args.cycles} циклов...")
        durations = []
        for _ in range(args.cycles):
            start = time.perf_counter()
            bus.read_positions()
            durations.append(time.perf_counter() - start)
        report("Чтение", durations, args.target_hz)

        if args.write:
            print("\nЗамер 2: чтение + запись. Моторы включат удержание в текущей позиции.")
            input("Убедитесь, что руке ничего не мешает, и нажмите ENTER (Ctrl+C — отмена)...")
            move_time_ms = int(1000 / args.target_hz)
            durations = []
            try:
                for _ in range(args.cycles):
                    start = time.perf_counter()
                    positions = bus.read_positions()
                    bus.write_positions(positions, time_ms=move_time_ms)
                    durations.append(time.perf_counter() - start)
            finally:
                input("Придержите руку и нажмите ENTER — удержание будет выключено...")
                bus.disable_torque()
            report("Чтение + запись", durations, args.target_hz)
        return 0
    except LXBusError as e:
        print(f"\nОшибка шины: {e}")
        return 1
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    sys.exit(main())
