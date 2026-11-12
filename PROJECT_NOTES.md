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
- Leader-рука пока не решена (варианты: LX с выключенным удержанием, STS3215,
  потенциометры + Arduino, телефон или клавиатура).

## Программное окружение (Windows)
- Глобальный Python 3.13: на нём работает `lx_setup.py` (pyserial стоит).
- Conda-окружение `lerobot` (Miniforge), Python 3.12, `lerobot` 0.6.2 с PyPI
  (`pip install "lerobot[feetech]"`).
- Не ставить `lerobot[all]`: зависимость `placo` не собирается на Windows.
- В cmd использовать двойные кавычки в командах pip.
- Работающий код протокола LX: папка `LewanSoul-master/src`
  (`lx_setup.py`: info / assign / test / set_pos / set_temp_limit / reset / scan).

## Подход к драйверу (предложение, ещё не утверждено)
Не наследоваться от `SerialMotorsBus` LeRobot (он работает через rustypot, а rustypot 1.13.1 поддерживает только Feetech и Dynamixel).
Вместо этого:
1. Тонкая обёртка `LXBus` над готовой библиотекой из `LewanSoul-master/src`:
   read_positions(), write_positions(), torque_on/off().
2. Свой класс `SO101LXFollower(Robot)` реализует только интерфейс Robot:
   connect, disconnect, calibrate, configure, get_observation, send_action,
   observation_features, action_features, is_connected, is_calibrated.
3. Калибровка хранится в штатном формате LeRobot (`MotorCalibration`):
   homing_offset = центр, range_min, range_max; перевод сырых значений
   в градусы делается внутри класса.
4. Оформить как плагин-пакет (префикс `lerobot_robot_...`), саму библиотеку не править.
Имена суставов как в SO-101: shoulder_pan, shoulder_lift, elbow_flex,
wrist_flex, (wrist_roll), gripper; ключи действий вида "shoulder_pan.pos".

## Ближайшие шаги
1. Сверить интерфейс `Robot` в установленной версии LeRobot 0.6.2.
2. Посмотреть API библиотеки в `LewanSoul-master/src`, написать `LXBus`.
3. Проверить чтение и запись позиций 5 моторов и замерить частоту цикла (цель ~30 Гц).
4. Написать класс робота и калибровку, проверить `lerobot-calibrate`.
5. Решить вопрос с leader и проверить `lerobot-teleoperate`.
6. Камеры (2 USB-камеры, 640×480, 30 fps), затем `lerobot-record`.
