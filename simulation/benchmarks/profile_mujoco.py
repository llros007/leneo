import os,sys,time,json,argparse
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'mujoco'))
from run import Simulation
from common.control import Control
from window import camera_default
import mujoco
p=argparse.ArgumentParser();p.add_argument('--quality',choices=['fast','full'],default='fast');args=p.parse_args()
s=Simulation(args.quality);c=Control();c.press('W');c.press('A')
t=time.perf_counter()
for _ in range(500):c.tick(s.dt);s.step(c)
step_ms=(time.perf_counter()-t)*2
camera=camera_default();camera.lookat[:]=s.data.qpos[:3]+[0,0,.5]
option=mujoco.MjvOption();option.geomgroup[0]=option.geomgroup[3]=0
with mujoco.Renderer(s.model,height=640,width=960) as r:
 def render():
  r.update_scene(s.data,camera=camera,scene_option=option)
  r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=args.quality=='full'
  r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
  r.render()
 for _ in range(5):render()
 t=time.perf_counter()
 for _ in range(30):render()
 render_ms=(time.perf_counter()-t)*1000/30
result=dict(quality=args.quality,unique_triangles=int(sum(s.model.mesh_facenum)),instanced_triangles=sum(int(s.model.mesh_facenum[s.model.geom_dataid[i]]) for i in range(s.model.ngeom) if s.model.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH),load_seconds=s.load_seconds,step_ms=step_ms,render_ms=render_ms)
print(json.dumps(result,indent=2));Path(f'simulation/outputs/mujoco_profile_{args.quality}.json').write_text(json.dumps(result,indent=2))
