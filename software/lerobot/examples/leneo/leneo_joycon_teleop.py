#!/usr/bin/env python
"""LeNeo real-robot teleop with Nintendo Joy-Con (official XLeRobot mapping).

Arm mapping matches ``xlerobot_mujoco_joycon_official.py`` / ``7_xlerobot_teleop_joycon.py``:
  * Right Joy-Con SR -> ARM mode (stick EE, R up, ZR down, Y/A gripper)
  * Right Joy-Con SL -> MOVE mode (X forward, B back, Y left, A right, R/ZR lift)

Requires the ``joyconrobotics`` package (XLeRobot ``software/`` tree on PYTHONPATH).

Example (conda env le312)::

    conda activate le312
    export JOYCON_SOFTWARE_DIR=/path/to/xlerobot/software
    cd /home/fishros/workspace/lerobot/lerobot
    PYTHONPATH=src python examples/leneo/leneo_joycon_teleop.py \\
        --robot.type=leneo \\
        --robot.port=/dev/ttyACM0 \\
        --robot.id=blue_follower_arm
"""

import logging
import math
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat

import numpy as np

from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.robots import leneo  # noqa: F401
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging

logger = logging.getLogger(__name__)

_EXAMPLES_LENEO_DIR = os.path.dirname(os.path.abspath(__file__))
if _EXAMPLES_LENEO_DIR not in sys.path:
    sys.path.insert(0, _EXAMPLES_LENEO_DIR)
from jh_foc_lift import LiftController

# ---------------------------------------------------------------------------
# Official SO101 kinematics (degrees I/O)
# ---------------------------------------------------------------------------
_ARM_L1, _ARM_L2 = 0.1159, 0.1350
_ARM_R_MAX = _ARM_L1 + _ARM_L2 - 0.002
_ARM_R_MIN = abs(_ARM_L1 - _ARM_L2) + 0.002
_ARM_X0, _ARM_Y0 = 0.1629, 0.1131


def _fit_ik_xy(x, y, prefer="y"):
    y = max(-_ARM_R_MAX, min(_ARM_R_MAX, y))
    x = max(0.02, x)
    r2 = x * x + y * y
    r_max2 = _ARM_R_MAX * _ARM_R_MAX
    r_min2 = _ARM_R_MIN * _ARM_R_MIN
    if r_min2 <= r2 <= r_max2:
        return x, y
    if prefer == "y":
        max_x2 = max(0.0, r_max2 - y * y)
        min_x2 = max(0.0, r_min2 - y * y)
        x2 = x * x
        if x2 > max_x2:
            x = math.sqrt(max_x2)
        elif x2 < min_x2:
            x = math.sqrt(min_x2) if min_x2 > 0 else 0.02
        return x, y
    max_y2 = max(0.0, r_max2 - x * x)
    min_y2 = max(0.0, r_min2 - x * x)
    y2 = y * y
    if y2 > max_y2:
        y = math.copysign(math.sqrt(max_y2), y) if y != 0 else 0.0
    elif y2 < min_y2:
        y = math.copysign(math.sqrt(min_y2), y) if y != 0 else math.sqrt(min_y2)
    return x, y


def _apply_fit_to_joy(joy, prefer="y"):
    x = _ARM_X0 + joy.position[0]
    y = _ARM_Y0 + joy.position[2]
    x, y = _fit_ik_xy(x, y, prefer=prefer)
    joy.position[0] = x - _ARM_X0
    joy.position[2] = y - _ARM_Y0


class SO101Kinematics:
    def __init__(self, l1=_ARM_L1, l2=_ARM_L2):
        self.l1 = l1
        self.l2 = l2

    def inverse_kinematics(self, x, y, l1=None, l2=None):
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2
        theta1_offset = math.atan2(0.028, 0.11257)
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
        r = math.sqrt(x**2 + y**2)
        if r > _ARM_R_MAX and r > 0:
            x, y = _fit_ik_xy(x, y, prefer="y")
            r = math.hypot(x, y)
        elif 0 < r < _ARM_R_MIN:
            x, y = _fit_ik_xy(x, y, prefer="y")
            r = math.hypot(x, y)
        cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        theta2 = math.pi - math.acos(cos_theta2)
        beta = math.atan2(y, x)
        gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        theta1 = beta + gamma
        joint2 = max(-0.1, min(3.45, theta1 + theta1_offset))
        joint3 = max(-0.2, min(math.pi, theta2 + theta2_offset))
        joint2_deg = 90.0 - math.degrees(joint2)
        joint3_deg = math.degrees(joint3) - 90.0
        return joint2_deg, joint3_deg


def _import_joycon_robotics(software_dir: str | None):
    candidates = []
    if software_dir:
        candidates.append(os.path.abspath(software_dir))
    env = os.environ.get("JOYCON_SOFTWARE_DIR")
    if env:
        candidates.append(os.path.abspath(env))
    candidates.extend(
        [
            os.path.expanduser("~/workspace/lerobot/XLeRobot/software"),
            os.path.expanduser("~/workspace/xlerobot/software"),
            os.path.expanduser("~/xlerobot/software"),
        ]
    )
    for path in candidates:
        if path and os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)
    try:
        from joyconrobotics import JoyconRobotics
    except ImportError as e:
        raise ImportError(
            "Cannot import joyconrobotics. Set JOYCON_SOFTWARE_DIR to the XLeRobot "
            "'software' folder (contains joyconrobotics). "
            "On Linux also install: pip install hidapi"
        ) from e
    return JoyconRobotics


def _list_joycon_hid() -> list[tuple]:
    import hid

    nintendo = []
    for dev in hid.enumerate(0, 0):
        if dev.get("vendor_id") != 0x057E:
            continue
        nintendo.append(
            (
                hex(dev.get("product_id", 0)),
                dev.get("product_string"),
                dev.get("serial") or dev.get("serial_number"),
            )
        )
    return nintendo


def _joycon_available_hid_or_evdev(side: str) -> str:
    """Return 'hid', 'evdev', or raise if neither works."""
    from joyconrobotics.device import get_L_ids, get_R_ids

    side = side.lower()
    if side not in ("left", "right"):
        raise ValueError(f"joycon.side must be 'left' or 'right', got {side!r}")
    ids = get_R_ids() if side == "right" else get_L_ids()
    if ids and ids[0][0] is not None:
        logging.info("Found %s Joy-Con (hidapi): %s", side, ids[0])
        return "hid"
    evdev_error = ""
    try:
        from evdev_joycon import find_joycon_evdev

        find_joycon_evdev(side)
        logging.info("Found %s Joy-Con (evdev / Bluetooth + hid_nintendo)", side)
        return "evdev"
    except Exception as exc:
        evdev_error = f"{type(exc).__name__}: {exc}"
    hid_list = _list_joycon_hid()
    lines = [
        f"未找到 {side} 手柄。请确认：",
        "  1) 已与电脑蓝牙配对（系统设置里显示已连接即可）",
        "  2) pip install hidapi evdev",
        "  3) 当前用户在 input 组: sudo usermod -aG input $USER 后重新登录",
        "  4) 若只有另一只手柄，可加 --joycon.side=left",
    ]
    if hid_list:
        lines.append("hidapi 枚举到的 Nintendo 设备：")
        for row in hid_list:
            lines.append(f"  product_id={row[0]} product_string={row[1]!r} serial={row[2]!r}")
    else:
        lines.append(
            "hidapi 为空但蓝牙已连接时属正常，脚本会走 evdev；若仍失败请检查 /dev/input/event* 权限。"
        )
    if evdev_error:
        lines.append(f"evdev: {evdev_error}")
    raise RuntimeError("\n".join(lines))


def make_fixed_axes_joycon_class(JoyconRobotics):
    """Subclass with official teleop stick / gripper mapping."""

    class FixedAxesJoyconRobotics(JoyconRobotics):
        enable_arm_stick = True

        def __init__(self, device, **kwargs):
            super().__init__(device, **kwargs)
            if self.joycon.is_right():
                self.joycon_stick_v_0 = 1900
                self.joycon_stick_h_0 = 2100
            else:
                self.joycon_stick_v_0 = 2300
                self.joycon_stick_h_0 = 2000
            self.gripper_speed = 0.4
            self.gripper_direction = 1
            self.gripper_min = 0
            self.gripper_max = 90
            self.last_gripper_button_state = 0
            self.enable_arm_stick = True

        def common_update(self):
                speed_scale = 0.001
                freeze_arm = self.joycon.is_right() and not self.enable_arm_stick

                if not freeze_arm:
                    joycon_stick_v = (
                        self.joycon.get_stick_right_vertical()
                        if self.joycon.is_right()
                        else self.joycon.get_stick_left_vertical()
                    )
                    v_th, v_range = 300, 1000
                    if abs(joycon_stick_v - self.joycon_stick_v_0) > v_th:
                        # evdev +Y is stick-up; +position[0] retracts this arm, so flip it.
                        # Pitch from the IMU steers that motion between reach (x) and height (z).
                        delta = -(joycon_stick_v - self.joycon_stick_v_0) / v_range * speed_scale * 0.5
                        pitch = float(self.orientation_rad[1])
                        self.position[0] += (
                            delta * math.cos(pitch) * self.dof_speed[0] * self.direction_reverse[0]
                        )
                        self.position[2] += (
                            delta * math.sin(pitch) * self.dof_speed[2] * self.direction_reverse[2]
                        )
                        _apply_fit_to_joy(self, prefer="x")

                    joycon_stick_h = (
                        self.joycon.get_stick_right_horizontal()
                        if self.joycon.is_right()
                        else self.joycon.get_stick_left_horizontal()
                    )
                    h_th, h_range = 300, 1000
                    if abs(joycon_stick_h - self.joycon_stick_h_0) > h_th:
                        k = (joycon_stick_h - self.joycon_stick_h_0) / h_range * speed_scale
                        self.position[1] += k * self.dof_speed[1] * self.direction_reverse[1]

                up = self.joycon.get_button_r() if self.joycon.is_right() else self.joycon.get_button_l()
                if self.joycon.is_right():
                    up = 0
                if up == 1:
                    self.position[2] += speed_scale * self.dof_speed[2] * self.direction_reverse[2]
                if not self.joycon.is_right():
                    down = (
                        self.joycon.get_button_l_stick() == 1 or self.joycon.get_button_left_sl() == 1
                    )
                    if down:
                        self.position[2] -= speed_scale * self.dof_speed[2] * self.direction_reverse[2]

                home = self.joycon.get_button_home() if self.joycon.is_right() else self.joycon.get_button_capture()
                if home == 1:
                    self.position = self.offset_position_m.copy()

                for event_type, status in self.button.events():
                    if (self.joycon.is_right() and event_type == "plus" and status == 1) or (
                        self.joycon.is_left() and event_type == "minus" and status == 1
                    ):
                        self.reset_button = 1
                        self.reset_joycon()
                    elif self.joycon.is_right() and event_type == "a":
                        self.next_episode_button = status
                    elif self.joycon.is_right() and event_type == "y":
                        self.restart_episode_button = status
                    else:
                        self.reset_button = 0

                if not self.joycon.is_right():
                    pressed = self.joycon.get_button_zl() == 1
                    if pressed and self.last_gripper_button_state == 0:
                        self.gripper_direction *= -1
                    self.last_gripper_button_state = pressed
                    if pressed:
                        nxt = self.gripper_state + self.gripper_direction * self.gripper_speed
                        if self.gripper_min <= nxt <= self.gripper_max:
                            self.gripper_state = nxt

                if self.joycon.is_right():
                    if self.next_episode_button == 1:
                        self.button_control = 1
                    elif self.restart_episode_button == 1:
                        self.button_control = -1
                    elif self.reset_button == 1:
                        self.button_control = 8
                    else:
                        self.button_control = 0
                return self.position, self.gripper_state, self.button_control

    return FixedAxesJoyconRobotics


def connect_joycon(side: str, **kwargs):
    JoyconRobotics = _import_joycon_robotics(None)
    FixedCls = make_fixed_axes_joycon_class(JoyconRobotics)
    from evdev_joycon import connect_fixed_axes_joycon

    return connect_fixed_axes_joycon(FixedCls, JoyconRobotics, side, **kwargs)


class OfficialTeleopArm:
    X0, Y0 = 0.1629, 0.1131
    Y_SCALE = 250.0

    def __init__(self, kinematics: SO101Kinematics):
        self.kinematics = kinematics
        self.zero()

    def zero(self):
        self.target_deg = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }

    def handle_joycon_input(self, joycon_pose, gripper_state):
        x, y, z, roll_, pitch_, yaw = joycon_pose
        pitch = -math.degrees(pitch_) + 10.0
        # Side tilt (roll) and twist about vertical (yaw) both turn the gripper.
        roll = math.degrees(roll_) + math.degrees(yaw)
        current_x = self.X0 + x
        current_y = self.Y0 + z
        self.target_deg["shoulder_pan"] = y * self.Y_SCALE
        try:
            j2, j3 = self.kinematics.inverse_kinematics(current_x, current_y)
            self.target_deg["shoulder_lift"] = j2
            self.target_deg["elbow_flex"] = j3
        except Exception as exc:
            logger.warning("IK failed: %s", exc)
        self.target_deg["wrist_flex"] = (
            -self.target_deg["shoulder_lift"] - self.target_deg["elbow_flex"] + pitch
        )
        self.target_deg["wrist_roll"] = roll
        self.target_deg["gripper"] = float(gripper_state)

    def as_leneo_action(self) -> dict[str, float]:
        return {f"arm_{motor}.pos": val for motor, val in self.target_deg.items()}


def _base_action_move_mode(joy_r, robot) -> dict[str, float]:
    """MOVE 模式差速：X 前进，B 后退，Y 左转，A 右转。"""
    speed = robot.speed_levels[robot.speed_index]
    linear = speed["linear"]
    angular = speed["angular"]
    jr = joy_r.joycon
    x_cmd = 0.0
    theta_cmd = 0.0
    if jr.get_button_x() == 1:
        x_cmd += linear
    if jr.get_button_b() == 1:
        x_cmd -= linear
    if jr.get_button_y() == 1:
        theta_cmd += angular
    if jr.get_button_a() == 1:
        theta_cmd -= angular
    return {"x.vel": x_cmd, "theta.vel": theta_cmd}


@dataclass
@dataclass
class JoyconTeleopConfig:
    side: str = "right"
    list_devices: bool = False


@dataclass
class LiftJoyconConfig:
    enabled: bool = True
    port: str = "/dev/ttyACM2"
    baud: int = 115200
    motor: str = "60"
    speed_rpm: float = 500.0


@dataclass
class LeneoJoyconTeleopConfig:
    robot: RobotConfig
    fps: int = 30
    joycon_software_dir: str | None = None
    arm_grip_step: float = 0.54
    arm_z_step: float = 0.0024
    joycon: JoyconTeleopConfig = field(default_factory=JoyconTeleopConfig)
    lift: LiftJoyconConfig = field(default_factory=LiftJoyconConfig)


@parser.wrap()
def main(cfg: LeneoJoyconTeleopConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.joycon_software_dir:
        os.environ["JOYCON_SOFTWARE_DIR"] = cfg.joycon_software_dir
    _import_joycon_robotics(cfg.joycon_software_dir)

    if cfg.joycon.list_devices:
        from joyconrobotics.device import get_device_ids, get_L_ids, get_R_ids

        print("Joy-Con (joyconrobotics):", get_device_ids())
        print("Right:", get_R_ids())
        print("Left:", get_L_ids())
        print("Raw HID Nintendo:", _list_joycon_hid())
        return

    side = cfg.joycon.side
    _joycon_available_hid_or_evdev(side)

    robot = make_robot_from_config(cfg.robot)
    arm = OfficialTeleopArm(SO101Kinematics())

    logging.info("Connecting %s Joy-Con ...", side)
    joy_r = connect_joycon(side, dof_speed=[2, 2, 2, 1, 1, 1])
    joy_r.enable_arm_stick = False
    input_mode = "move"
    sr_prev = sl_prev = 0

    lift: LiftController | None = None
    if cfg.lift.enabled:
        try:
            logging.info("Connecting JH_FOC lift on %s ...", cfg.lift.port)
            lift = LiftController(
                port=cfg.lift.port,
                baud=cfg.lift.baud,
                motor_type=cfg.lift.motor,
                speed_rpm=cfg.lift.speed_rpm,
            )
            logging.info("Lift motor ready (MOVE: R=up ZR=down, release=stop).")
        except Exception as exc:
            logging.warning("Lift disabled: could not open %s (%s)", cfg.lift.port, exc)
            lift = None

    robot.connect()
    logging.info(
        "LeNeo connected. SL=MOVE (X 前进, B 后退, Y 左转, A 右转, R/ZR 升降), "
        "SR=ARM (stick + R/ZR EE + Y/A grip). Ctrl+C to stop."
    )

    try:
        while True:
            loop_start = time.perf_counter()
            _, grip_r, btn_r = joy_r.get_control()
            jr = joy_r.joycon
            sr = jr.get_button_right_sr()
            sl = jr.get_button_right_sl()
            if sl == 1 and sl_prev == 0:
                input_mode = "move"
                joy_r.enable_arm_stick = False
                logging.info("MODE: MOVE")
            if sr == 1 and sr_prev == 0:
                input_mode = "arm"
                joy_r.enable_arm_stick = True
                logging.info("MODE: ARM")
            sr_prev, sl_prev = sr, sl

            action: dict[str, float] = {"x.vel": 0.0, "theta.vel": 0.0}

            if btn_r == 8:
                arm.zero()
                joy_r.position = joy_r.offset_position_m.copy()
                joy_r._twist_rad = 0.0
                logging.info("Plus: arm targets reset to 0 deg")
            elif input_mode == "arm":
                now = time.perf_counter()
                dt = now - getattr(joy_r, "_twist_t", now)
                joy_r._twist_t = now
                # Library yaw gain (gyro/86) barely moves. Integrate Z gyro directly.
                gz = float(joy_r.gyro.gyro_in_rad[0][2])
                joy_r._twist_rad = getattr(joy_r, "_twist_rad", 0.0) + gz * dt
                orientation = list(joy_r.orientation_rad)
                orientation[2] = joy_r._twist_rad
                joy_r.orientation_rad = orientation
                if jr.get_button_r() == 1:
                    joy_r.position[2] += cfg.arm_z_step
                if jr.get_button_zr() == 1:
                    joy_r.position[2] -= cfg.arm_z_step
                _apply_fit_to_joy(joy_r, prefer="y")
                pose_r = list(joy_r.position) + list(joy_r.orientation_rad)
                arm.handle_joycon_input(pose_r, grip_r)
                g = arm.target_deg["gripper"]
                if jr.get_button_y() == 1:
                    g = min(90.0, g + cfg.arm_grip_step)
                if jr.get_button_a() == 1:
                    g = max(0.0, g - cfg.arm_grip_step)
                arm.target_deg["gripper"] = g
                joy_r.gripper_state = g
                action.update(arm.as_leneo_action())
            else:
                action.update(_base_action_move_mode(joy_r, robot))

            if lift is not None:
                lift.update(
                    jr.get_button_r() == 1,
                    jr.get_button_zr() == 1,
                    active=input_mode == "move",
                )

            robot.send_action(action)
            precise_sleep(max(1 / cfg.fps - (time.perf_counter() - loop_start), 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        if lift is not None:
            try:
                lift.close()
            except Exception:
                pass
        try:
            joy_r.disconnect()
        except Exception:
            pass
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    register_third_party_plugins()
    main()
