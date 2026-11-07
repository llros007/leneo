"""Record the actual MuJoCo task at a fixed simulation-time frame rate."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import sys,json,math,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'mujoco'))
from run import Simulation
from common.control import Control,ROOT
from common.video import VideoWriter
from common.demo_camera import recording_default
from common.pick_orange import PickOrange
from demo_camera import DemoCamera,camera_default
import mujoco
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=ROOT/'outputs/videos/mujoco_pick_orange.mp4');parser.add_argument('--seconds',type=float,default=62);args=parser.parse_args()
s=Simulation();c=Control();task=PickOrange(s,c);director=DemoCamera(camera_default(),overrides={'default':recording_default()})
out=args.output;video=VideoWriter(out,1280,800,30,'MuJoCo')
option=mujoco.MjvOption();option.geomgroup[0]=option.geomgroup[3]=0
with mujoco.Renderer(s.model,height=800,width=1280) as renderer:
    try:
        while video.next_time<=args.seconds+1e-8:
            target=video.next_time
            while s.time+1e-9<target:
                c.tick(s.dt);task.tick();s.step(c);director.update(task.phase,s.time)
            renderer.update_scene(s.data,camera=director.camera,scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
            video.write(renderer.render(),s.time,task.phase)
            if video.frames%300==0:print(f'Video: {s.time:.1f}/{args.seconds:g} seconds',flush=True)
        assert task.success and task.done,task.report()
    finally:video.close()
report=dict(video=video.report(),task=task.report(),camera_events=director.events)
out.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(video.report()),flush=True)
