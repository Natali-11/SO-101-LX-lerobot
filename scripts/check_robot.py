"""Проверка класса робота на настоящей руке (после lerobot-calibrate).

Пример (из корня репозитория, в окружении conda `lerobot`):
    python scripts\\check_robot.py --port COM3 --id my_lx_arm
    python scripts\\check_robot.py --port COM3 --id my_lx_arm --joint elbow_flex --delta 15

Что делает:
  1. показывает положение суставов в градусах (захват — в процентах);
  2. после подтверждения сдвигает ОДИН сустав на --delta в сторону середины
     его диапазона и возвращает обратно, сравнивая команду с результатом;
  3. замеряет частоту цикла «наблюдение + действие», как в телеуправлении.
Во время шагов 2–3 все моторы держат положение; в конце они отпускаются.
"""

import argparse
import statistics
import sys
import time

from lerobot_robot_so101_lx import SO101LXFollower, SO101LXFollowerConfig

SETTLE_S = 1.5
TOLERANCE = 3.0  # допустимое расхождение команды и результата, градусы (для захвата — %)


def show(title: str, obs: dict) -> None:
    print(title)
    for key, value in obs.items():
        unit = "%" if key.startswith("gripper") else "°"
        print(f"  {key.removesuffix('.pos'):14s} {value:8.1f} {unit}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Проверка робота SO-101 LX на железе")
    parser.add_argument("--port", required=True, help="порт, например COM3")
    parser.add_argument("--id", required=True, help="то же имя, что в lerobot-calibrate --robot.id")
    parser.add_argument("--joint", default="wrist_flex", help="какой сустав сдвинуть")
    parser.add_argument("--delta", type=float, default=10.0, help="на сколько сдвинуть, градусы (захват — %%)")
    parser.add_argument("--cycles", type=int, default=100, help="число циклов в замере частоты")
    args = parser.parse_args()

    robot = SO101LXFollower(SO101LXFollowerConfig(port=args.port, id=args.id))
    key = f"{args.joint}.pos"
    if key not in robot.action_features:
        print(f"Нет сустава '{args.joint}'. Есть: {[k.removesuffix('.pos') for k in robot.action_features]}")
        return 1
    if not robot.is_calibrated:
        print(f"Нет калибровки для id '{args.id}' ({robot.calibration_fpath}).")
        print(f"Сначала: lerobot-calibrate --robot.type=so101_lx_follower --robot.port={args.port} --robot.id={args.id}")
        return 1

    robot.connect(calibrate=False)
    try:
        start = robot.get_observation()
        show("\nТекущее положение:", start)

        # Двигаемся в сторону середины диапазона, чтобы не упереться в упор.
        middle = 50.0 if args.joint == "gripper" else 0.0
        delta = args.delta if start[key] <= middle else -args.delta
        print(f"\nСустав '{args.joint}' сдвинется на {delta:+.0f} и вернётся. Остальные будут держать положение.")
        input("Придержите руку, уберите всё лишнее и нажмите ENTER (Ctrl+C — отмена)...")

        ok = True
        for label, target_value in (("туда", start[key] + delta), ("обратно", start[key])):
            target = dict(start)
            target[key] = target_value
            sent = robot.send_action(target)
            time.sleep(SETTLE_S)
            reached = robot.get_observation()[key]
            error = reached - sent[key]
            verdict = "ок" if abs(error) <= TOLERANCE else "РАСХОЖДЕНИЕ"
            ok = ok and abs(error) <= TOLERANCE
            print(f"  {label:8s} команда {sent[key]:7.1f}, получилось {reached:7.1f}, разница {error:+5.1f}  {verdict}")

        print(f"\nЗамер частоты цикла «наблюдение + действие», {args.cycles} циклов (рука стоит на месте)...")
        durations = []
        for _ in range(args.cycles):
            t0 = time.perf_counter()
            robot.get_observation()
            robot.send_action(start)
            durations.append(time.perf_counter() - t0)
        ms = sorted(d * 1000 for d in durations)
        mean = statistics.fmean(ms)
        print(f"  среднее {mean:.1f} мс, 95% циклов до {ms[int(0.95 * (len(ms) - 1))]:.1f} мс, худший {ms[-1]:.1f} мс")
        print(f"  -> {1000 / mean:.0f} Гц (цель 30 Гц)")

        print("\nИтог:", "движение совпало с командой." if ok else "движение НЕ совпало с командой — пришлите этот вывод.")
        return 0 if ok else 1
    finally:
        try:
            input("\nПридержите руку и нажмите ENTER — моторы будут отпущены...")
        finally:
            robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
