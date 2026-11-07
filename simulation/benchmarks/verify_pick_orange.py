"""Exercise the complete task, actual release contacts, and stable final placement."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import sys,json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'mujoco'))
from run import Simulation
from pick_orange import PickOrange
from window import camera_default
from common.control import Control,ROOT
from task_scene import ORANGE,PLACE,TABLE_TOP
from grasp_geometry import measure
import mujoco
from PIL import Image
s=Simulation()
# A nearby fruit outside the jaw plane must not be picked up by a distance-only weld.
s.data.qpos[s.joints['right_gripper']]=.51683;s.mj.mj_forward(s.model,s.data)
R=s.data.site_xmat[s.grip_site].reshape(3,3)
s.data.qpos[s.orange_q:s.orange_q+3]=s.data.site_xpos[s.grip_site]+R@np.array([0,.023,0])
s.mj.mj_forward(s.model,s.data)
try:s.grasp(True)
except RuntimeError as exc:assert 'not between both fingers' in str(exc),str(exc)
else:raise AssertionError('Assisted grasp accepted a fruit outside the fingers')
s.mj.mj_resetDataKeyframe(s.model,s.data,0);s.mj.mj_forward(s.model,s.data);s.grasp_geometry=None
c=Control();task=PickOrange(s,c);out=ROOT/'outputs/pick_orange';out.mkdir(parents=True,exist_ok=True)
op=mujoco.MjvOption();op.geomgroup[0]=op.geomgroup[3]=0;camera=camera_default()
phases={'pregrasp':'initial','grasp':'grasp','carry':'lifted','approach-bar':'carrying','place':'above_bar','open':'released','complete':'final'}
saved=set();peak=0.;contacts=set();attachment_distance=None;max_jaw_gap=0.;max_local_drift=0.;max_rotation_drift=0.
with mujoco.Renderer(s.model,height=640,width=960) as renderer:
    for step in range(16000):
        if task.phase in phases and task.phase not in saved:
            renderer.update_scene(s.data,camera=camera,scene_option=op);renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
            Image.fromarray(renderer.render()).save(out/(phases[task.phase]+'.png'));saved.add(task.phase)
            if task.phase in ['grasp','carry','reverse']:
                close=mujoco.MjvCamera();close.lookat[:]=s.data.qpos[s.orange_q:s.orange_q+3]+[0,0,.025]
                close.distance=.28;close.azimuth=110;close.elevation=-15
                renderer.update_scene(s.data,camera=close,scene_option=op)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                Image.fromarray(renderer.render()).save(out/(phases[task.phase]+'_close.png'))
            if task.phase=='grasp':attachment_distance=float(np.linalg.norm(s.data.site_xpos[s.grip_site]-s.data.qpos[s.orange_q:s.orange_q+3]))
        if task.done:break
        c.tick(s.dt);task.tick();s.step(c)
        peak=max(peak,float(s.data.qpos[s.orange_q+2]))
        if s.attached and step%100==0:
            geometry=measure(s)
            assert geometry['between_fingers'],(task.phase,geometry)
            max_jaw_gap=max(max_jaw_gap,max(abs(g) for g in geometry['jaw_surface_gaps_m']))
            R=s.data.site_xmat[s.grip_site].reshape(3,3)
            local=R.T@(s.data.qpos[s.orange_q:s.orange_q+3]-s.data.site_xpos[s.grip_site])
            max_local_drift=max(max_local_drift,float(np.linalg.norm(local-s.grasp_offset)))
            rel=R.T@s.data.xmat[s.model.body('orange').id].reshape(3,3)
            max_rotation_drift=max(max_rotation_drift,float(np.linalg.norm(rel-s.grasp_rotation)))
        for contact in s.data.contact:
            a,b=int(contact.geom1),int(contact.geom2)
            if s.model.geom(a).name=='orange_contact' or s.model.geom(b).name=='orange_contact':
                contacts.add(tuple(sorted([s.model.geom(a).name,s.model.geom(b).name])))
    assert task.done and task.success,task.report()
    assert not s.attached and s.data.eq_active[s.grasp_eq]==0
    assert max_local_drift<.002 and max_rotation_drift<.03
    for point in [[-.9,-.45],[.8,-.45]]:
        assert not s.navigation.collides(point,0), 'Removed dining table still blocks the robot'
    assert peak>TABLE_TOP+.08
    assert ('ground','orange_contact') in contacts
    assert ('orange_contact','orange_table_support') in contacts
    assert s.navigation.blocked_steps==0
    assert not any(w.number for w in s.data.warning)
    p0=s.data.qpos[s.orange_q:s.orange_q+3].copy()
    for _ in range(500):s.step(c)
    assert np.linalg.norm(s.data.qpos[s.orange_q:s.orange_q+3]-p0)<.001
    result=dict(max_jaw_surface_gap_m=max_jaw_gap,max_grasp_local_drift_m=max_local_drift,max_grasp_rotation_drift=max_rotation_drift,dining_table_removed=True,passed=True,completion_seconds=float(task.since),grasp_distance_m=attachment_distance,peak_height_m=peak,contacts=sorted(contacts),settle_displacement_m=float(np.linalg.norm(s.data.qpos[s.orange_q:s.orange_q+3]-p0)),task=task.report())
    (out/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
