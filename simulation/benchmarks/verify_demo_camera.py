"""Run the actual pick/place task and capture its three automatic camera cues."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'mujoco'))
from run import Simulation
from pick_orange import PickOrange
from common.control import Control,ROOT
from demo_camera import DemoCamera,camera_default,camera_preset
import mujoco
from PIL import Image

# Manual edits survive repeated updates in the same cue, including during a blend.
probe=DemoCamera(camera_default());probe.update('floor-grasp',1);probe.update('close',1.2)
probe.manual();probe.camera.distance=1.7;probe.camera.azimuth=135
manual=probe.values(probe.camera).copy();probe.update('grasp',2)
assert np.array_equal(manual,probe.values(probe.camera))
probe.update('carry',3);probe.update('reverse',4)
assert np.allclose(probe.values(probe.camera),probe.values(camera_default()))

s=Simulation();c=Control();task=PickOrange(s,c);director=DemoCamera(camera_default())
out=ROOT/'outputs/demo_camera';out.mkdir(parents=True,exist_ok=True)
option=mujoco.MjvOption();option.geomgroup[0]=option.geomgroup[3]=0
saved=set();last=director.values(director.camera);max_delta=np.zeros(6)
with mujoco.Renderer(s.model,height=640,width=960) as renderer:
    for _ in range(15000):
        c.tick(s.dt);task.tick();s.step(c);director.update(task.phase,s.data.time)
        current=director.values(director.camera);max_delta=np.maximum(max_delta,np.abs(current-last));last=current
        assert np.isfinite(current).all() and current[3]>.1
        key={'close':'pick','approach-bar':'default','release':'place','complete':'final'}.get(task.phase)
        if key and key not in saved and not director.transition:
            assert np.allclose(current,director.values(camera_preset(director.cue)))
            renderer.update_scene(s.data,camera=director.camera,scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
            Image.fromarray(renderer.render()).save(out/(key+'.png'));saved.add(key)
        if task.done:break
assert task.success and saved=={'pick','default','place','final'}
assert [(e['phase'],e['view']) for e in director.events]==[('floor-grasp','pick'),('carry','default'),('above-bar','place')]
assert max_delta[3]<.04 and max_delta[4]<1.3 and np.max(max_delta[:3])<.04,max_delta
result=dict(passed=True,events=director.events,manual_adjustment_preserved=True,smooth_transition_seconds=director.duration,max_camera_delta_per_10ms=max_delta.tolist(),task_success=task.success)
(out/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
