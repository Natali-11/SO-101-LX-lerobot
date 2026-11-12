# Проект: SO-101 на сервоприводах LewanSoul LX + LeRobot

## Цель
Повторить роборуку SO-101 (Hugging Face LeRobot) как можно ближе к оригиналу,
но на сервоприводах LewanSoul LX вместо Feetech STS3215, и пользоваться
инструментами LeRobot: телеуправление, запись датасетов, обучение и запуск политик.

## Железо
- 5 сервоприводов LewanSoul LX (шина TTL, 115200, шкала 0–1000 = 0–240°, 1 шаг ≈ 0,24°).
- Плата LewanSoul подключается по USB, в Windows это COM-порт (не /dev/ttyUSB0).
- Новые сервоприводы имеют ID = 1. ID назначать строго по одному мотору на шине.
- Питание сервоприводов 6–8,4 В, отдельно от USB.
- В оригинале SO-101 6 моторов на руку. Вариант с 5 моторами: без wrist_roll.
- Leader-рука: склоняемся к энкодерам AS5600 в корпусах-заглушках вместо моторов
  (другие варианты: LX с выключенным удержанием, STS3215, потенциометры).

## Программное окружение (Windows)
- Глобальный Python 3.13: на нём работает `lx_setup.py` (pyserial стоит).
- Conda-окружение `lerobot` (Miniforge), Python 3.12, `lerobot` 0.6.2 установлен
  из исходников в режиме editable: клон в `C:\Users\pomol\lerobot`,
  коммит 772bc21 (`pip install -e ".[feetech]"`). Не делать там `git pull` без нужды.
- Не ставить `lerobot[all]`: зависимость `placo` не собирается на Windows.
- В cmd использовать двойные кавычки в командах pip.
- Работающий код протокола LX: папка `LewanSoul-master/src`
  (`lx_setup.py`: info / assign / test / set_pos / set_temp_limit / reset / scan).

## Устройство драйвера (пакет `lerobot_robot_so101_lx`)
Шина LeRobot (`SerialMotorsBus`) работает через rustypot, а rustypot 1.13.1
поддерживает только Feetech и Dynamixel, поэтому от неё не наследуемся.
1. `lx_bus.py` — `LXBus`, тонкая обёртка над `lewansoul_lx16a.py` (копия из
   `LewanSoul-master/src`): read_positions(), write_positions(),
   enable_torque(), disable_torque(). Сырые шаги 0–1000.
2. `so101_lx_follower.py` — `SO101LXFollower(Robot)`, тип `so101_lx_follower`.
   Суставы в градусах (0° в середине откалиброванного диапазона), захват 0–100.
3. Калибровка — штатный файл LeRobot (`MotorCalibration`): range_min, range_max
   в шагах; drive_mode = 1 разворачивает сустав; homing_offset не используется.
   В моторы ничего не пишется.
4. Саму библиотеку LeRobot не правим. `lerobot-setup-motors` не поддерживается:
   ID назначаются через `lx_setup.py assign`.
Имена суставов как в SO-101: shoulder_pan, shoulder_lift, elbow_flex,
wrist_flex, (wrist_roll), gripper; ключи действий вида "shoulder_pan.pos".
Тесты без железа: `python -m pytest tests` (имитация моторов в `tests/test_lx_bus.py`).

## Состояние
- Сделано и проверено на имитации: `LXBus`, `SO101LXFollower`, калибровка,
  запуск через `lerobot-calibrate`.
- На настоящих моторах НЕ проверено ничего из нового кода.

## Ближайшие шаги
1. В лаборатории: `python scripts\check_bus.py --port COMx --ids ...` (и с `--write`),
   замерить частоту цикла (цель ~30 Гц).
2. Проверить `lerobot-calibrate --robot.type=so101_lx_follower` на руке.
3. Leader: планируется на магнитных энкодерах AS5600 + ESP32 в корпусах-заглушках;
   нужен плагин `lerobot_teleoperator_...`, затем `lerobot-teleoperate`.
4. Камеры (2 USB-камеры, 640×480, 30 fps), затем `lerobot-record`.
