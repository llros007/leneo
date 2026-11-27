"""Check dual wrist-camera frames, kinematics, projection and native rendering."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import sys,json,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'mujoco'))
from run import Simulation
from common.orange_layout import GRIP

D=ROOT/'assets/leneo_description'
config=json.loads((D/'config/wrist_cameras.json').read_text())
for filename in ['leneo.urdf','leneo_ros.urdf']:
    tree=ET.parse(D/'urdf'/filename).getroot()
    links={e.get('name'):e for e in tree.findall('link')}
    joints={e.find('child').get('link'):e for e in tree.findall('joint')}
    assert len(links)==len(tree.findall('link'))
    assert len(joints)==len(links)-1
    for mesh in tree.findall('.//mesh'):
        path=mesh.get('filename')
        target=(D/path.removeprefix('package://leneo_description/')) if path.startswith('package://') else (D/'urdf'/path)
        assert target.is_file(),target
    for side in ['left','right']:
        name=side+'_wrist_camera'
        child=name+'_optical_frame';chain=[]
        while child!=side+'_gripper_link':
            j=joints[child];assert j.get('type')=='fixed'
            o=j.find('origin');T=np.eye(4)
            T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
            T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
            chain.insert(0,T);child=j.find('parent').get('link')
        T=np.eye(4)
        for part in chain:T=T@part
        q=config['camera_opengl_quat_wxyz'];R=Rotation.from_quat(q[1:]+q[:1]).as_matrix()
        assert np.allclose(T[:3,3],config['camera_position_in_gripper_m'],atol=1e-10)
        assert np.allclose(T[:3,:3],R@np.diag([1,-1,-1]),atol=1e-10)

s=Simulation();m,d=s.model,s.data;mj=s.mj
out=ROOT/'outputs/wrist_cameras';out.mkdir(parents=True,exist_ok=True)
initial=d.qpos.copy();snapshots={};worst=0.
q=config['camera_opengl_quat_wxyz'];expected=Rotation.from_quat(q[1:]+q[:1]).as_matrix()
for name,changes in [('home',{}),('left_rotated',{'left_wrist_roll':.7}),('right_flexed',{'right_wrist_flex':.4,'right_wrist_roll':-.5})]:
    d.qpos[:]=initial
    for joint,value in changes.items():d.qpos[s.joints[joint]]=value
    mj.mj_forward(m,d)
    snapshots[name]={}
    for side in ['left','right']:
        camera=m.camera(side+'_wrist_camera').id
        body=m.body(side+'_gripper_link').id
        parent_R=d.xmat[body].reshape(3,3)
        local_pos=parent_R.T@(d.cam_xpos[camera]-d.xpos[body])
        local_R=parent_R.T@d.cam_xmat[camera].reshape(3,3)
        error=float(np.max(np.abs(local_pos-config['camera_position_in_gripper_m'])))
        worst=max(worst,error)
        assert error<1e-9 and np.allclose(local_R,expected,atol=1e-9)
        assert abs(m.cam_fovy[camera]-config['fovy_deg'])<1e-9
        tool=m.body(side+'_tool0').id
        pinch=d.xpos[tool]+d.xmat[tool].reshape(3,3)@GRIP
        point=d.cam_xmat[camera].reshape(3,3).T@(pinch-d.cam_xpos[camera])
        depth=-point[2];assert depth>0
        focal=config['resolution'][1]/(2*np.tan(np.deg2rad(config['fovy_deg'])/2))
        pixel=np.array(config['resolution'])/2+focal*np.array([point[0],-point[1]])/depth
        assert np.all(pixel>0) and np.all(pixel<config['resolution']),pixel
        snapshots[name][side]=(d.cam_xpos[camera].copy(),d.cam_xmat[camera].copy())
assert not np.allclose(snapshots['home']['left'][1],snapshots['left_rotated']['left'][1])
assert np.allclose(snapshots['home']['right'][0],snapshots['left_rotated']['right'][0])
assert np.allclose(snapshots['home']['right'][1],snapshots['left_rotated']['right'][1])
assert not np.allclose(snapshots['home']['right'][1],snapshots['right_flexed']['right'][1])
d.qpos[:]=initial;mj.mj_forward(m,d)
option=mj.MjvOption();option.geomgroup[0]=option.geomgroup[3]=0
with mj.Renderer(m,height=config['resolution'][1],width=config['resolution'][0]) as renderer:
    for side in ['left','right']:
        renderer.update_scene(d,camera=side+'_wrist_camera',scene_option=option)
        pixels=renderer.render()
        assert pixels.shape==(config["resolution"][1],config["resolution"][0],3) and pixels.std()>5
        Image.fromarray(pixels).save(out/(side+'_wrist_camera.png'))
report=dict(passed=True,cameras=config['camera_names'],fovy_deg=config['fovy_deg'],resolution=config['resolution'],max_local_position_error_m=worst,urdf_variants_checked=2,independent_wrist_motion=True)
(out/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
