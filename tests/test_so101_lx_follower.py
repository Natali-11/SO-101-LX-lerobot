"""Тесты класса робота без железа. Нужен установленный LeRobot."""

import sys

import pytest
import serial

pytest.importorskip("lerobot")

from lerobot.motors import MotorCalibration  # noqa: E402
from lerobot.robots import make_robot_from_config  # noqa: E402
from test_lx_bus import FakeSerial  # noqa: E402

import lerobot_robot_so101_lx.so101_lx_follower as robot_module  # noqa: E402
from lerobot_robot_so101_lx import SO101LXFollower, SO101LXFollowerConfig  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "gripper"]


@pytest.fixture
def fake(monkeypatch):
    port = FakeSerial({1: 500, 2: 500, 3: 500, 4: 500, 5: 300})
    monkeypatch.setattr(serial, "Serial", lambda *args, **kwargs: port)
    return port


def make_robot(tmp_path, **kwargs) -> SO101LXFollower:
    config = SO101LXFollowerConfig(port="COM_TEST", id="test", calibration_dir=tmp_path, **kwargs)
    return SO101LXFollower(config)


def calibrated_robot(tmp_path, **kwargs) -> SO101LXFollower:
    robot = make_robot(tmp_path, **kwargs)
    # Суставы 200..800 (середина 500, ход 144°), захват 300..600.
    robot.calibration = {
        name: MotorCalibration(id=i, drive_mode=0, homing_offset=0, range_min=200, range_max=800)
        for i, name in enumerate(JOINTS[:-1], start=1)
    }
    robot.calibration["gripper"] = MotorCalibration(
        id=5, drive_mode=0, homing_offset=0, range_min=300, range_max=600
    )
    return robot


def sweep_joints(fake, monkeypatch, low=150, high=850):
    """Имитация оператора: на каждом опросе ENTER суставы уходят к одному краю, потом к другому."""
    calls = {"n": 0}

    def fake_enter_pressed():
        calls["n"] += 1
        if calls["n"] == 1:
            fake.positions.update({i: low for i in fake.positions})
        elif calls["n"] == 2:
            fake.positions.update({i: high for i in fake.positions})
        return calls["n"] >= 3

    monkeypatch.setattr(robot_module, "enter_pressed", fake_enter_pressed)
    monkeypatch.setattr("builtins.input", lambda *args: "")


def test_features_and_factory(tmp_path, fake):
    robot = make_robot_from_config(SO101LXFollowerConfig(port="COM_TEST", id="t", calibration_dir=tmp_path))
    assert isinstance(robot, SO101LXFollower)
    assert list(robot.action_features) == [f"{j}.pos" for j in JOINTS]
    assert robot.observation_features == robot.action_features
    assert not robot.is_calibrated and not robot.is_connected


def test_calibrate_saves_and_reloads(tmp_path, fake, monkeypatch):
    sweep_joints(fake, monkeypatch)
    robot = make_robot(tmp_path)
    robot.connect(calibrate=False)
    fake.torque.update({i: True for i in fake.torque})
    robot.calibrate()
    assert not any(fake.torque.values())  # калибровка отпускает моторы
    assert robot.is_calibrated
    assert robot.calibration["elbow_flex"].range_min == 150
    assert robot.calibration["elbow_flex"].range_max == 850
    robot.disconnect()

    reloaded = make_robot(tmp_path)
    assert reloaded.is_calibrated
    assert reloaded.calibration == robot.calibration


def test_calibrate_rejects_joint_that_did_not_move(tmp_path, fake, monkeypatch):
    monkeypatch.setattr(robot_module, "enter_pressed", lambda: True)
    monkeypatch.setattr("builtins.input", lambda *args: "")
    robot = make_robot(tmp_path)
    robot.connect(calibrate=False)
    with pytest.raises(RuntimeError, match="не двигались"):
        robot.calibrate()
    assert not robot.is_calibrated
    assert not robot.calibration_fpath.exists()


def test_observation_units(tmp_path, fake):
    robot = calibrated_robot(tmp_path)
    robot.connect()
    fake.positions.update({1: 500, 2: 600, 3: 400, 5: 450})
    obs = robot.get_observation()
    assert obs["shoulder_pan.pos"] == pytest.approx(0.0)
    assert obs["shoulder_lift.pos"] == pytest.approx(24.0)  # 100 шагов * 0,24°
    assert obs["elbow_flex.pos"] == pytest.approx(-24.0)
    assert obs["gripper.pos"] == pytest.approx(50.0)
    assert set(obs) == set(robot.observation_features)


def test_send_action_clips_to_calibrated_range(tmp_path, fake):
    robot = calibrated_robot(tmp_path)
    robot.connect()
    action = {"shoulder_pan.pos": 12.0, "shoulder_lift.pos": 500.0, "elbow_flex.pos": -500.0,
              "wrist_flex.pos": 0.0, "gripper.pos": 150.0}
    sent = robot.send_action(action)
    assert fake.positions == {1: 550, 2: 800, 3: 200, 4: 500, 5: 600}
    assert sent["shoulder_pan.pos"] == pytest.approx(12.0)
    assert sent["shoulder_lift.pos"] == pytest.approx(72.0)
    assert sent["elbow_flex.pos"] == pytest.approx(-72.0)
    assert sent["gripper.pos"] == pytest.approx(100.0)
    assert all(fake.torque.values())


def test_speed_limit_stretches_far_moves(tmp_path, fake):
    robot = calibrated_robot(tmp_path, move_time_ms=40, max_speed_deg_s=180.0)
    robot.connect()
    # Первый раз: от 500 до 800 = 72° при 180°/с -> 400 мс; близкая цель -> обычные 40 мс.
    robot.send_action({"shoulder_pan.pos": 72.0, "shoulder_lift.pos": 1.0})
    assert fake.move_times[1] == 400
    assert fake.move_times[2] == 40
    # Дальше отсчёт идёт от прошлой цели: шаг в 2,4° укладывается в 40 мс.
    robot.send_action({"shoulder_pan.pos": 69.6})
    assert fake.move_times[1] == 40


def test_max_relative_target(tmp_path, fake):
    robot = calibrated_robot(tmp_path, max_relative_target=5.0)
    robot.connect()
    sent = robot.send_action({"shoulder_pan.pos": 60.0})
    assert sent["shoulder_pan.pos"] == pytest.approx(5.0, abs=0.13)
    assert fake.positions[1] == 521


def test_drive_mode_inverts_direction(tmp_path, fake):
    robot = calibrated_robot(tmp_path)
    robot.calibration["shoulder_pan"].drive_mode = 1
    robot.calibration["gripper"].drive_mode = 1
    robot.connect()
    robot.send_action({"shoulder_pan.pos": 24.0, "gripper.pos": 100.0})
    assert fake.positions[1] == 400 and fake.positions[5] == 300
    obs = robot.get_observation()
    assert obs["shoulder_pan.pos"] == pytest.approx(24.0)
    assert obs["gripper.pos"] == pytest.approx(100.0)


def test_requires_calibration_and_connection(tmp_path, fake):
    robot = make_robot(tmp_path)
    with pytest.raises(Exception):
        robot.get_observation()  # не подключён
    robot.connect(calibrate=False)
    with pytest.raises(RuntimeError, match="не откалиброван"):
        robot.get_observation()
    with pytest.raises(RuntimeError, match="не откалиброван"):
        robot.send_action({"shoulder_pan.pos": 0.0})
    robot.disconnect()
    assert not robot.is_connected


def test_lerobot_calibrate_command(tmp_path, fake, monkeypatch):
    """Штатная команда lerobot-calibrate находит плагин и проходит калибровку целиком."""
    from lerobot.scripts import lerobot_calibrate

    sweep_joints(fake, monkeypatch)
    monkeypatch.setattr(
        sys,
        "argv",
        ["lerobot-calibrate", "--robot.type=so101_lx_follower", "--robot.port=COM_TEST",
         "--robot.id=cli", f"--robot.calibration_dir={tmp_path}"],
    )
    lerobot_calibrate.calibrate()
    assert (tmp_path / "cli.json").is_file()
    assert not fake.is_open
