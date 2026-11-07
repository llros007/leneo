"""Engine-independent LeNeo controls: SI units and original URDF joint names."""
import json
import math
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / 'assets/leneo_description'
PARAMS = json.loads((DESCRIPTION / 'config/model_parameters.json').read_text())
HOME = json.loads((DESCRIPTION / 'config/home_positions.json').read_text())
ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
HELP = 'Hold WASD/arrows: drive+turn; release: brake | Q/E or R/F: lift | Tab: arm, 1..6: joint, J/K: move | H: home | Esc: quit'


class Control:
    def __init__(self):
        import xml.etree.ElementTree as ET
        self.limits = {}
        for joint in ET.parse(DESCRIPTION / 'urdf/leneo.urdf').getroot().findall('joint'):
            limit = joint.find('limit')
            if limit is not None and 'lower' in limit.attrib:
                self.limits[joint.attrib['name']] = (float(limit.attrib['lower']), float(limit.attrib['upper']))
        self.targets = dict(HOME)
        self.side = 'left'
        self.selected = 0
        self.linear = self.angular = 0.0
        self.last_drive = 0.0
        self.quit = False
        self.command_mtime = None
        self.held = set()
        self.keyboard_active = False

    @staticmethod
    def normalize_key(key):
        key=key.upper().removeprefix('KEY_').removeprefix('NUMPAD_')
        return {'UP':'W','DOWN':'S','LEFT':'A','RIGHT':'D','SPACEBAR':'SPACE'}.get(key,key)

    def press(self,key):
        key=self.normalize_key(key)
        self.held.add(key)
        if key not in ['W','A','S','D','Q','E','R','F','J','K','SPACE']:
            self.key(key)

    def release(self,key):
        self.held.discard(self.normalize_key(key))

    def clear_keys(self):
        self.held.clear()
        self.linear=self.angular=0.0
        self.keyboard_active=False

    def tick(self,dt):
        active=bool(self.held & {'W','A','S','D','SPACE'})
        if active or self.keyboard_active:
            self.linear=0.15*(('W' in self.held)-('S' in self.held))
            self.angular=0.55*(('A' in self.held)-('D' in self.held))
            self.last_drive=time.monotonic()
        self.keyboard_active=active
        if 'SPACE' in self.held:
            self.linear=self.angular=0.0
            return
        lift=(bool(self.held & {'Q','R'})-bool(self.held & {'E','F'}))*0.05*dt
        if lift:self.set_joint('lift_joint',self.targets['lift_joint']+lift)
        arm=(('J' in self.held)-('K' in self.held))*0.6*dt
        if arm:
            name=f'{self.side}_{ARM_JOINTS[self.selected]}'
            self.set_joint(name,self.targets[name]+arm)

    def key(self, key):
        key = key.upper()
        if key in ['ESCAPE', 'ESC']: self.quit = True
        elif key in ['W', 'S', 'A', 'D']:
            self.linear = {'W': 0.15, 'S': -0.15}.get(key, 0.0)
            self.angular = {'A': 0.55, 'D': -0.55}.get(key, 0.0)
            self.last_drive = time.monotonic()
        elif key in ['SPACE', ' ']: self.linear = self.angular = 0.0
        elif key in ['R', 'F']: self.set_joint('lift_joint', self.targets['lift_joint'] + (0.01 if key == 'R' else -0.01))
        elif key == 'TAB':
            self.side = 'right' if self.side == 'left' else 'left'; print('Selected', self.side, ARM_JOINTS[self.selected], flush=True)
        elif key in '123456' and len(key) == 1:
            self.selected = int(key) - 1; print('Selected', self.side, ARM_JOINTS[self.selected], flush=True)
        elif key in ['J', 'K']:
            name = f'{self.side}_{ARM_JOINTS[self.selected]}'
            self.set_joint(name, self.targets[name] + (0.05 if key == 'J' else -0.05))
        elif key == 'H':
            self.targets = dict(HOME); self.linear = self.angular = 0.0

    def set_joint(self, name, value):
        if name not in self.limits or name.endswith('wheel_joint') or 'caster' in name:
            raise ValueError(f'Not a position-controlled joint: {name}')
        value = float(value)
        if not math.isfinite(value): raise ValueError('Joint target must be finite')
        lo, hi = self.limits[name]
        self.targets[name] = max(lo, min(hi, value))

    def read_command(self, path):
        """JSON: {linear: m/s, angular: rad/s, joints: {joint_name: SI_target}}.

        Drive expires after 0.5 s; rewrite the file periodically for continuous motion.
        Parse/validate fully before applying a command.
        """
        if not path: return
        path = Path(path)
        if not path.exists(): return
        stamp = path.stat().st_mtime_ns
        if stamp == self.command_mtime: return
        command = json.loads(path.read_text())
        linear, angular = float(command.get('linear', 0)), float(command.get('angular', 0))
        if not all(math.isfinite(v) for v in [linear, angular]): raise ValueError('Velocity must be finite')
        old = dict(self.targets)
        try:
            for name, value in command.get('joints', {}).items(): self.set_joint(name, value)
        except Exception:
            self.targets = old
            raise
        self.linear = max(-0.25, min(0.25, linear))
        self.angular = max(-0.8, min(0.8, angular))
        self.last_drive = time.monotonic(); self.command_mtime = stamp

    def wheels(self):
        if time.monotonic() - self.last_drive > 0.5: self.linear = self.angular = 0.0
        radius, track = PARAMS['wheel_radius_m'], PARAMS['wheel_track_m']
        left = (self.linear - self.angular * track / 2) / radius
        right = (self.linear + self.angular * track / 2) / radius
        scale = max(1.0, abs(left) / 6, abs(right) / 6)
        return left / scale, right / scale

    def demo(self, elapsed):
        # Small deterministic excitation suitable for a smoke test in the aisle.
        self.set_joint('lift_joint', 0.025 * (1 - math.cos(min(elapsed, 4) * math.pi / 4)))
        self.set_joint('left_shoulder_pan', HOME['left_shoulder_pan'] + 0.12 * math.sin(elapsed))
        self.set_joint('right_shoulder_pan', HOME['right_shoulder_pan'] - 0.12 * math.sin(elapsed))
        self.linear = 0.06 if 1 < elapsed < 2 else 0.0
        self.angular = 0.2 if 3 < elapsed < 4 else 0.0
        self.last_drive = time.monotonic()
