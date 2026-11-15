"""Linux Bluetooth Joy-Con via evdev (hid_nintendo driver).

When the kernel claims the Joy-Con, hidapi often sees nothing even though
Bluetooth shows connected. This matches XLeRobot ``ProjectJoyconReader``.
"""

from __future__ import annotations

import glob
import logging
import math
import os
import select
import threading
import time

logger = logging.getLogger(__name__)


def find_joycon_evdev(side: str = "right"):
    from evdev import InputDevice, list_devices

    side = side.lower()
    # hid_nintendo: "Joy-Con Right"; hid-generic: "Joy-Con (R)"
    wants = ("Right", "(R)") if side == "right" else ("Left", "(L)")
    main_dev = None
    imu_dev = None
    for path in list_devices():
        d = InputDevice(path)
        name = d.name or ""
        if "Joy-Con" not in name or not any(w in name for w in wants):
            continue
        if "IMU" in name:
            imu_dev = d
        else:
            main_dev = d
    if main_dev is None:
        raise RuntimeError(f"evdev: no {side} Joy-Con (non-IMU) device found")
    return main_dev, imu_dev


def _find_hidraw(side: str) -> str | None:
    """hid-generic exposes Joy-Con buttons only inside report 0x30, not evdev."""
    product = "2007" if side == "right" else "2006"
    for node in sorted(glob.glob("/sys/class/hidraw/hidraw*")):
        try:
            uevent = open(os.path.join(node, "device", "uevent"), encoding="utf-8").read()
        except OSError:
            continue
        if "Joy-Con" in uevent and product in uevent:
            return "/dev/" + os.path.basename(node)
    return None


class _HidrawJoyConShim:
    """Decode Nintendo standard input report 0x30 from /dev/hidraw.

    The generic HID driver maps only report 0x3F, which the Joy-Con does not
    stream. Buttons and the stick therefore look dead until this report is read.
    """

    def __init__(self, path: str, side: str):
        self._side = side
        self._lock = threading.Lock()
        self._right = 0
        self._shared = 0
        self._left = 0
        self._stick = (2048, 2048)
        self._fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self._timer = 0
        self._send(0x03, bytes([0x30]))  # standard full input reports
        self._send(0x30, bytes([0x01]))  # player LED 1 solid
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="joycon-hidraw", daemon=True)
        self._thread.start()

    def _send(self, subcommand: int, data: bytes) -> None:
        rumble = bytes([0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40])
        packet = bytes([0x01, self._timer & 0x0F]) + rumble + bytes([subcommand]) + data
        self._timer = (self._timer + 1) & 0x0F
        try:
            os.write(self._fd, packet)
        except OSError as exc:
            logger.warning("Joy-Con hidraw write failed: %s", exc)

    def _loop(self) -> None:
        while not self._stop.is_set():
            ready, _, _ = select.select([self._fd], [], [], 0.2)
            if not ready:
                continue
            try:
                data = os.read(self._fd, 256)
            except OSError:
                continue
            if len(data) < 12 or data[0] != 0x30:
                continue
            if self._side == "right":
                stick = data[9:12]
            else:
                stick = data[6:9]
            x = stick[0] | ((stick[1] & 0x0F) << 8)
            y = (stick[1] >> 4) | (stick[2] << 4)
            with self._lock:
                self._right = data[3]
                self._shared = data[4]
                self._left = data[5]
                self._stick = (x, y)

    def _bit(self, which: str, mask: int) -> int:
        with self._lock:
            value = {"right": self._right, "shared": self._shared, "left": self._left}[which]
        return 1 if value & mask else 0

    def is_right(self) -> bool:
        return self._side == "right"

    def is_left(self) -> bool:
        return self._side == "left"

    def _pump(self) -> None:
        """State is already refreshed by the hidraw reader thread."""

    def _close(self) -> None:
        self._stop.set()
        try:
            os.close(self._fd)
        except OSError:
            pass

    def get_stick_right_horizontal(self, sample_rate: int = 0) -> int:
        del sample_rate
        with self._lock:
            return self._stick[0]

    def get_stick_right_vertical(self, sample_rate: int = 0) -> int:
        del sample_rate
        with self._lock:
            return self._stick[1]

    def get_stick_left_horizontal(self, sample_rate: int = 0) -> int:
        return self.get_stick_right_horizontal(sample_rate)

    def get_stick_left_vertical(self, sample_rate: int = 0) -> int:
        return self.get_stick_right_vertical(sample_rate)

    def get_button_y(self) -> int:
        return self._bit("right", 0x01)

    def get_button_x(self) -> int:
        return self._bit("right", 0x02)

    def get_button_b(self) -> int:
        return self._bit("right", 0x04)

    def get_button_a(self) -> int:
        return self._bit("right", 0x08)

    def get_button_right_sr(self) -> int:
        return self._bit("right", 0x10)

    def get_button_right_sl(self) -> int:
        return self._bit("right", 0x20)

    def get_button_r(self) -> int:
        return self._bit("right", 0x40)

    def get_button_zr(self) -> int:
        return self._bit("right", 0x80)

    def get_button_minus(self) -> int:
        return self._bit("shared", 0x01)

    def get_button_plus(self) -> int:
        return self._bit("shared", 0x02)

    def get_button_r_stick(self) -> int:
        return self._bit("shared", 0x04)

    def get_button_l_stick(self) -> int:
        return self._bit("shared", 0x08)

    def get_button_home(self) -> int:
        return self._bit("shared", 0x10)

    def get_button_capture(self) -> int:
        return self._bit("shared", 0x20)

    def get_button_left_sr(self) -> int:
        return self._bit("left", 0x10)

    def get_button_left_sl(self) -> int:
        return self._bit("left", 0x20)

    def get_button_l(self) -> int:
        return self._bit("left", 0x40)

    def get_button_zl(self) -> int:
        return self._bit("left", 0x80)


class _EvdevJoyConShim:
    """Subset of joyconrobotics.joycon.JoyCon used by FixedAxesJoyconRobotics."""

    def __init__(self, dev, side: str):
        from evdev import ecodes

        self._dev = dev
        self._ecodes = ecodes
        self._side = side
        self._btn: dict[int, bool] = {}
        self._lock = threading.Lock()
        self._abs_rx = 0
        self._abs_ry = 0
        self.product_id = 0x2007 if side == "right" else 0x2006
        self._refresh_axes()

    def _refresh_axes(self) -> None:
        """Kernel stick is already centered: 0 at rest, ±32767 at full throw."""
        rx = self._dev.absinfo(self._ecodes.ABS_RX)
        ry = self._dev.absinfo(self._ecodes.ABS_RY)
        if rx is not None:
            self._abs_rx = int(rx.value)
        if ry is not None:
            self._abs_ry = int(ry.value)

    def _pump(self) -> None:
        with self._lock:
            while True:
                event = self._dev.read_one()
                if event is None:
                    break
                if event.type == self._ecodes.EV_KEY:
                    self._btn[event.code] = event.value != 0
            self._refresh_axes()

    @staticmethod
    def _stick_to_hid(v: int) -> int:
        return int((int(v) + 32767) / 65534 * 4095)

    def is_right(self) -> bool:
        return self._side == "right"

    def is_left(self) -> bool:
        return self._side == "left"

    def get_stick_right_vertical(self) -> int:
        self._pump()
        return self._stick_to_hid(self._abs_ry)

    def get_stick_right_horizontal(self) -> int:
        self._pump()
        return self._stick_to_hid(self._abs_rx)

    def get_stick_left_vertical(self) -> int:
        return self.get_stick_right_vertical()

    def get_stick_left_horizontal(self) -> int:
        return self.get_stick_right_horizontal()

    def _pressed(self, code) -> int:
        self._pump()
        return 1 if self._btn.get(code) else 0

    def get_button_y(self) -> int:
        # hid-nintendo: BTN_WEST = Nintendo Y
        return self._pressed(self._ecodes.BTN_WEST)

    def get_button_x(self) -> int:
        # hid-nintendo: BTN_NORTH = Nintendo X
        return self._pressed(self._ecodes.BTN_NORTH)

    def get_button_b(self) -> int:
        # hid-nintendo: BTN_SOUTH = Nintendo B (Linux alias BTN_A is south, not Nintendo A)
        return self._pressed(self._ecodes.BTN_SOUTH)

    def get_button_a(self) -> int:
        # hid-nintendo: BTN_EAST = Nintendo A
        return self._pressed(self._ecodes.BTN_EAST)

    def get_button_r(self) -> int:
        return self._pressed(self._ecodes.BTN_TR)

    def get_button_zr(self) -> int:
        return self._pressed(self._ecodes.BTN_TR2)

    def get_button_right_sr(self) -> int:
        # Right Joy-Con SR is BTN_TL2, not the shoulder R (BTN_TR).
        return self._pressed(self._ecodes.BTN_TL2)

    def get_button_right_sl(self) -> int:
        # Right Joy-Con SL is BTN_TL.
        return self._pressed(self._ecodes.BTN_TL)

    def get_button_home(self) -> int:
        return self._pressed(self._ecodes.BTN_MODE)

    def get_button_plus(self) -> int:
        return self._pressed(self._ecodes.BTN_START)

    def get_button_capture(self) -> int:
        return 0

    def get_button_l(self) -> int:
        return 0

    def get_button_zl(self) -> int:
        return 0

    def get_button_l_stick(self) -> int:
        return self._pressed(self._ecodes.BTN_THUMBR)

    def get_button_r_stick(self) -> int:
        return self._pressed(self._ecodes.BTN_THUMBR)

    def _close(self) -> None:
        try:
            self._dev.close()
        except Exception:
            pass


class _EvdevButtonEvents:
    def __init__(self, shim: _EvdevJoyConShim):
        self._shim = shim
        self._prev_plus = 0
        self._prev_a = 0
        self._prev_y = 0

    def events(self):
        self._shim._pump()
        plus = self._shim.get_button_plus()
        a = self._shim.get_button_a()
        y = self._shim.get_button_y()
        out = []
        if plus == 1 and self._prev_plus == 0:
            out.append(("plus", 1))
        if a != self._prev_a:
            out.append(("a", a))
        if y != self._prev_y:
            out.append(("y", y))
        self._prev_plus = plus
        self._prev_a = a
        self._prev_y = y
        return out


class _EvdevGyroShim:
    """Feeds AttitudeEstimator from Joy-Con IMU evdev node."""

    def __init__(self, imu_dev):
        from evdev import ecodes

        self._dev = imu_dev
        self._ecodes = ecodes
        self._lock = threading.Lock()
        self._gyro_in_rad = [[0.0, 0.0, 0.0]]
        self._accel_in_g = [[0.0, 0.0, 0.0]]
        self._ax = self._ay = self._az = 0
        self._gx = self._gy = self._gz = 0
        # hid-nintendo: accel counts / 4096 = g; gyro counts / 14247 = deg/s.
        self._accel_per_g = 4096.0
        self._gyro_per_dps = 14247.0
        self._pump()

    @property
    def gyro_in_rad(self):
        # Real JoyCon refreshes IMU in its own thread. This shim must re-read on use.
        self._pump()
        return self._gyro_in_rad

    @property
    def accel_in_g(self):
        self._pump()
        return self._accel_in_g

    def _read_axis(self, code: int) -> int:
        info = self._dev.absinfo(code)
        return 0 if info is None else int(info.value)

    def _pump(self) -> None:
        if self._dev is None:
            return
        with self._lock:
            while True:
                event = self._dev.read_one()
                if event is None:
                    break
            e = self._ecodes
            self._ax = self._read_axis(e.ABS_X)
            self._ay = self._read_axis(e.ABS_Y)
            self._az = self._read_axis(e.ABS_Z)
            self._gx = self._read_axis(e.ABS_RX)
            self._gy = self._read_axis(e.ABS_RY)
            self._gz = self._read_axis(e.ABS_RZ)
            dps_to_rad = math.pi / 180.0
            # hid-nintendo Z points up. AttitudeEstimator uses atan2(ay, -az) and
            # expects Z down when the buttons face up, otherwise roll stays clamped.
            self._accel_in_g[0] = [
                self._ax / self._accel_per_g,
                self._ay / self._accel_per_g,
                -self._az / self._accel_per_g,
            ]
            self._gyro_in_rad[0] = [
                self._gx / self._gyro_per_dps * dps_to_rad,
                self._gy / self._gyro_per_dps * dps_to_rad,
                -self._gz / self._gyro_per_dps * dps_to_rad,
            ]

    def calibrate(self) -> None:
        self._pump()
        time.sleep(0.5)
        self._pump()

    def reset_orientation(self) -> None:
        pass


def build_evdev_joycon_robotics(JoyconRobotics, side: str, **kwargs):
    """Return JoyconRobotics instance with evdev-backed joycon/gyro/button."""
    main_dev, imu_dev = find_joycon_evdev(side)
    logger.info("Joy-Con evdev: %s (imu=%s)", main_dev.name, imu_dev.name if imu_dev else None)

    hidraw = _find_hidraw(side) if ("(R)" in main_dev.name or "(L)" in main_dev.name) else None
    if hidraw is not None:
        logger.info("Joy-Con hid-generic: reading buttons from %s", hidraw)
        shim = _HidrawJoyConShim(hidraw, side)
    else:
        shim = _EvdevJoyConShim(main_dev, side)
    gyro = _EvdevGyroShim(imu_dev)
    button = _EvdevButtonEvents(shim)

    obj = JoyconRobotics.__new__(JoyconRobotics)
    obj.joycon_id = (0x057E, 0x2007 if side == "right" else 0x2006, main_dev.phys or "evdev")
    obj.joycon = shim
    obj.gyro = gyro
    obj.button = button
    obj.lerobot = kwargs.get("lerobot", False)
    obj.pitch_down_double = kwargs.get("pitch_down_double", False)
    obj.rotation_filter_alpha_rate = kwargs.get("rotation_filter_alpha_rate", 1)

    from joyconrobotics.joyconrobotics import AttitudeEstimator

    obj.orientation_sensor = AttitudeEstimator(
        common_rad=kwargs.get("common_rad", True),
        lerobot=obj.lerobot,
        pitch_down_double=obj.pitch_down_double,
        # AttitudeEstimator multiplies this by 0.05. The library passes 1 here.
        lowpassfilter_alpha_rate=obj.rotation_filter_alpha_rate,
    )
    obj.without_rest_init = kwargs.get("without_rest_init", False)
    obj.gripper_open = kwargs.get("gripper_open", 1.0)
    obj.gripper_close = kwargs.get("gripper_close", 0.0)
    obj.gripper_state = kwargs.get("gripper_state", 1.0)
    obj.position = list(kwargs.get("offset_position_m", [0.0, 0.0, 0.0]))
    obj.offset_position_m = list(kwargs.get("offset_position_m", [0.0, 0.0, 0.0]))
    obj.orientation_rad = list(kwargs.get("offset_euler_rad", [0.0, 0.0, 0.0]))
    obj.offset_euler_rad = list(kwargs.get("offset_euler_rad", [0.0, 0.0, 0.0]))
    obj.dof_speed = list(kwargs.get("dof_speed", [1, 1, 1, 1, 1, 1]))
    obj.direction_reverse = list(kwargs.get("direction_reverse", [1, 1, 1]))
    obj.euler_reverse = list(kwargs.get("euler_reverse", [1, 1, 1]))
    obj.horizontal_stick_mode = kwargs.get("horizontal_stick_mode", "y")
    obj.if_close_y = kwargs.get("close_y", False)
    obj.if_limit_dof = kwargs.get("limit_dof", False)
    obj.glimit = kwargs.get(
        "glimit",
        [[0.125, -0.4, 0.046, -3.1, -1.5, -1.57], [0.380, 0.4, 0.23, 3.1, 1.5, 1.57]],
    )
    obj.pure_xz = kwargs.get("pure_xz", True)
    obj.change_down_to_gripper = kwargs.get("change_down_to_gripper", False)
    obj.gripper_toggle_button = 0
    obj.direction_vector = []
    obj.direction_vector_right = []
    obj.yaw_diff = 0.0
    obj.posture = obj.offset_position_m.copy()
    obj.reset_button = 0
    obj.next_episode_button = 0
    obj.restart_episode_button = 0
    obj.button_control = 0
    obj.running = True
    obj.lock = threading.Lock()
    if not obj.without_rest_init:
        obj.reset_joycon()
    obj.thread = threading.Thread(target=obj.solve_loop, daemon=True)
    obj.thread.start()
    return obj


def connect_fixed_axes_joycon(FixedCls, JoyconRobotics, side: str, **kwargs):
    from joyconrobotics.device import get_L_ids, get_R_ids

    ids_fn = get_R_ids if side == "right" else get_L_ids
    ids = ids_fn()
    if ids and ids[0][0] is not None:
        logger.info("Joy-Con hidapi: %s", ids[0])
        return FixedCls(side, **kwargs)

    logger.info("hidapi 未找到 Joy-Con，使用 evdev（蓝牙 + hid_nintendo）")
    base = build_evdev_joycon_robotics(JoyconRobotics, side, **kwargs)
    base.common_update = FixedCls.common_update.__get__(base, JoyconRobotics)
    base.enable_arm_stick = True
    # Kernel already zeros the stick. Capture that rest point instead of hid raw centers.
    if side == "right":
        base.joycon_stick_v_0 = base.joycon.get_stick_right_vertical()
        base.joycon_stick_h_0 = base.joycon.get_stick_right_horizontal()
    else:
        base.joycon_stick_v_0 = base.joycon.get_stick_left_vertical()
        base.joycon_stick_h_0 = base.joycon.get_stick_left_horizontal()
    logger.info(
        "Joy-Con stick rest v=%s h=%s (ARM mode only, press SR)",
        base.joycon_stick_v_0,
        base.joycon_stick_h_0,
    )
    base.gripper_speed = 0.4
    base.gripper_direction = 1
    base.gripper_min = 0
    base.gripper_max = 90
    base.last_gripper_button_state = 0
    return base
