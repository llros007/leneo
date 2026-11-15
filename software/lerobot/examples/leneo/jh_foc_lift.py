"""JH_FOC_42V3 lift motor for LeNeo (from ``motor/motor_keyboard_control.py``)."""

import os
import sys


def _motor_package_dir() -> str:
    candidates = [
        os.path.join(os.path.expanduser("~"), "ai", "leneo", "motor"),
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "motor")),
    ]
    for path in candidates:
        if os.path.isfile(os.path.join(path, "motor_keyboard_control.py")):
            return path
    return candidates[-1]


def _import_jh_foc():
    motor_dir = _motor_package_dir()
    if motor_dir not in sys.path:
        sys.path.insert(0, motor_dir)
    from motor_keyboard_control import DEFAULT_MOTOR, MOTOR_PRESETS, JHFocMotor

    return JHFocMotor, MOTOR_PRESETS, DEFAULT_MOTOR


class LiftController:
    """Velocity-mode lift: hold R up / ZR down (MOVE mode only)."""

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        motor_type: str = "60",
        speed_rpm: float = 80.0,
    ):
        JHFocMotor, MOTOR_PRESETS, default_motor = _import_jh_foc()
        motor_type = motor_type or default_motor
        preset = MOTOR_PRESETS[motor_type]
        self.speed_rpm = abs(speed_rpm)
        self.motor = JHFocMotor(port, baud=baud)
        self.motor.motor_name = preset["name"]
        self.motor.log_enabled = False
        self.motor.enter_speed_mode(current=float(preset["current"]), accel=float(preset["acc"]))
        self._last_cmd: float | None = None

    def update(self, up: bool, down: bool, active: bool, speed_rpm: float | None = None) -> None:
        speed = self.speed_rpm if speed_rpm is None else abs(speed_rpm)
        if not active:
            cmd = 0.0
        elif up and not down:
            cmd = speed
        elif down and not up:
            cmd = -speed
        else:
            cmd = 0.0
        if cmd != self._last_cmd:
            self.motor.set_speed(cmd)
            self._last_cmd = cmd

    def stop(self) -> None:
        self.update(False, False, active=True)

    def close(self) -> None:
        self.motor.close()
