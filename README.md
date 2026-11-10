# SO-101-LX-lerobot

Роборука SO-101 (Hugging Face LeRobot) на сервоприводах LewanSoul LX
вместо Feetech STS3215.

Цель — пользоваться инструментами LeRobot (телеуправление, запись датасетов,
обучение и запуск политик) с рукой на моторах LX.

## Состояние

Проект в начале пути. Сейчас в репозитории:

- `LewanSoul-master/` — сторонний код протокола LX (`src/lx_setup.py`:
  info / assign / test / set_pos / set_temp_limit / reset / scan).
- `CLAUDE.md` — описание железа, окружения и плана работ.

Планируется: обёртка `LXBus`, класс робота `SO101LXFollower` и оформление
в виде плагина LeRobot (`lerobot_robot_...`).

## Железо

- 5 сервоприводов LewanSoul LX (шина TTL, 115200 бод, шкала 0–1000 = 0–240°).
- Плата LewanSoul, подключение по USB (в Windows — COM-порт).
- Питание сервоприводов 6–8,4 В, отдельно от USB.

## Окружение

- Windows, conda-окружение `lerobot` (Python 3.12), `pip install "lerobot[feetech]"`.
- Для `lx_setup.py` нужен `pyserial`.
