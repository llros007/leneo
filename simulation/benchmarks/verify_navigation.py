"""Check combined keys, release braking, movement distance and swept obstacles."""
from pathlib import Path
import sys,json,math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.control import Control
from common.navigation import Navigation
c=Control();n=Navigation();start=n.position.copy()
c.press('W')
for _ in range(100):c.tick(.01);n.step(c,.01)
assert abs(n.position[1]-start[1]-.15)<1e-9
c.press('A')
for _ in range(100):c.tick(.01);n.step(c,.01)
assert n.yaw>math.pi/2+.5 and np.linalg.norm(n.position[:2]-start[:2])>.28
c.release('A');c.release('W');c.tick(.01);stopped=n.position.copy()
for _ in range(100):c.tick(.01);n.step(c,.01)
assert np.array_equal(n.position,stopped)
c.press('Q')
for _ in range(100):c.tick(.01);n.step(c,.01)
assert abs(n.joints['lift_joint']-.05)<1e-9
c.release('Q');c.press('KEY_3');assert c.selected==2
c.clear_keys();n=Navigation();c.press('W')
for _ in range(3000):c.tick(.01);n.step(c,.01)
assert n.blocked_steps>0
assert not n.collides(n.position[:2],n.yaw)
result=dict(passed=True,one_second_forward_m=.15,combined_keys=True,release_stop=True,lift_50mm=True,obstacle_blocks=n.blocked_steps,obstacle_stop=n.position.tolist())
Path('simulation/outputs/navigation_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
