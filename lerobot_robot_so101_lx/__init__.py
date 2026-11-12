"""Плагин LeRobot: рука SO-101 на сервоприводах LewanSoul LX."""

import importlib.util

from .lx_bus import LXBus, LXBusError

__all__ = ["LXBus", "LXBusError"]

# Шина работает и без LeRobot (достаточно pyserial), робот — только вместе с ним.
if importlib.util.find_spec("lerobot") is not None:
    from .config_so101_lx_follower import SO101LXFollowerConfig
    from .so101_lx_follower import SO101LXFollower

    __all__ += ["SO101LXFollower", "SO101LXFollowerConfig"]
