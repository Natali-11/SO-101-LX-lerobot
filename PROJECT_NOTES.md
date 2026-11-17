# Проект: SO-101 на сервоприводах LewanSoul LX + LeRobot

## Цель
Повторить роборуку SO-101 (Hugging Face LeRobot) как можно ближе к оригиналу,
но на сервоприводах LewanSoul LX вместо Feetech STS3215, и пользоваться
инструментами LeRobot: телеуправление, запись датасетов, обучение и запуск политик.

## Железо
- 6 сервоприводов LewanSoul LX (шина TTL, 115200, шкала 0–1000 = 0–240°, 1 шаг ≈ 0,24°).
  ID: 1 shoulder_pan, 2 shoulder_lift, 3 elbow_flex, 4 wrist_flex, 5 wrist_roll, 6 gripper.
- Плата LewanSoul подключается по USB, в Windows это COM-порт (не /dev/ttyUSB0).
- Новые сервоприводы имеют ID = 1. ID назначать строго по одному мотору на шине.
- Питание сервоприводов отдельно от USB. Блок даёт 8,4 В (одно время было 9,4 В —
  проверить маркировку моторов, для не-HV версий предел 8,4 В).
- Плата в лаборатории — COM4.
- Leader-рука: склоняемся к энкодерам AS5600 (6 шт. + запас) + ESP32 в корпусах-заглушках
  вместо моторов (другие варианты: LX с выключенным удержанием, STS3215, потенциометры).

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
   enable_torque(), disable_torque(). Сырые шаги 0–1000. Позиция читается
   одним обращением к порту (быстрее, чем через библиотеку).
2. `so101_lx_follower.py` — `SO101LXFollower(Robot)`, тип `so101_lx_follower`.
   Суставы в градусах (0° в середине откалиброванного диапазона), захват 0–100 %.
   Защиты в send_action: обрезка по калибровке, max_relative_target, предел
   скорости 180°/с, защита захвата от перегрева (если губки упёрлись дольше 0,5 с,
   цель подтягивается к фактическому положению).
3. Калибровка — штатный файл LeRobot (`MotorCalibration`): range_min, range_max
   в шагах; drive_mode = 1 разворачивает сустав; homing_offset не используется.
   В моторы ничего не пишется. Файл: %USERPROFILE%\.cache\huggingface\lerobot\
   calibration\robots\so101_lx_follower\<id>.json
4. Саму библиотеку LeRobot не правим. `lerobot-setup-motors` не поддерживается:
   ID назначаются через `scripts/center_motors.py --set-id`.
Имена суставов как в SO-101: shoulder_pan, shoulder_lift, elbow_flex,
wrist_flex, wrist_roll, gripper; ключи действий вида "shoulder_pan.pos".

## Скрипты
- `scripts/center_motors.py` — назначить ID (`--set-id`), выставить моторы на 500
  и держать, пока надеваются качалки.
- `scripts/check_bus.py` — связь с моторами и частота цикла (`--write`).
- `scripts/check_robot.py` — сдвиг одного сустава (`--joint`) и сравнение с командой.
- Тесты без железа: `python -m pytest tests` (имитация моторов в `tests/test_lx_bus.py`).

## Сборка руки (правило)
Мотор на 500 → сустав в середине своего хода → надеть качалку. Тогда весь ход
сустава помещается в шкалу мотора (240°). Признак ошибки: в калибровке у сустава
с упорами range_min = 0 или range_max = 1000.

## Состояние
- Проверено на 5 моторах (без захвата): шина, калибровка, движение wrist_flex
  совпало с командой (ошибка < 1°). Частота: чтение 32 мс, чтение+запись 37 мс
  → 27 Гц на 5 моторах; на 6 ожидается ~22 Гц.
- Найдено: качалка shoulder_lift стояла со сдвигом (калибровка давала 0–700, потом
  ложные 0–1000), elbow_flex у края шкалы. Решение — пересборка с центровкой.
- Не проверено на железе: захват и его защита, быстрое чтение позиции в LXBus.

## Ближайшие шаги
1. Назначить новому мотору ID 6, выставить все моторы на 500, пересобрать руку.
2. `lerobot-calibrate --robot.type=so101_lx_follower --robot.port=COM4 --robot.id=my_lx_arm`,
   проверить таблицу (нет 0 и 1000 у суставов с упорами).
3. `check_robot.py` для каждого сустава, включая gripper.
4. Частота: замер с новым lx_bus.py; если мало — Latency Timer адаптера = 1 мс,
   иначе работать на 25 Гц (`--fps=25`).
5. Leader на энкодерах + плагин `lerobot_teleoperator_...`, затем `lerobot-teleoperate`.
6. Камеры (2 USB-камеры, 640×480, 30 fps), затем `lerobot-record`.
