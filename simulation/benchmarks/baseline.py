import os,sys,time,json
os.environ['MUJOCO_GL']='egl'
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"mujoco"))
from build_scene import build
t=time.perf_counter();m=mujoco.MjModel.from_xml_path(str(build()));load=time.perf_counter()-t
d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,0)
counts=sorted([(m.mesh(i).name,int(m.mesh_facenum[i])) for i in range(m.nmesh)],key=lambda x:-x[1])
t=time.perf_counter()
for _ in range(1000):mujoco.mj_step(m,d)
step=(time.perf_counter()-t)/1000
opt=mujoco.MjvOption();opt.geomgroup[0]=opt.geomgroup[3]=0
cam=mujoco.MjvCamera();cam.lookat[:]=[-1.4,-.5,.55];cam.distance=1.45;cam.azimuth=-138;cam.elevation=-34
with mujoco.Renderer(m,height=640,width=960) as r:
 for _ in range(3):r.update_scene(d,camera=cam,scene_option=opt);r.render()
 t=time.perf_counter()
 for _ in range(20):r.update_scene(d,camera=cam,scene_option=opt);r.render()
 render=(time.perf_counter()-t)/20
result=dict(load_seconds=load,unique_triangles=int(sum(m.mesh_facenum)),instanced_triangles=sum(int(m.mesh_facenum[m.geom_dataid[i]]) for i in range(m.ngeom) if m.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH),step_ms=step*1000,render_ms=render*1000,contacts=d.ncon,largest_meshes=counts[:15]);print(json.dumps(result,indent=2));Path('simulation/outputs/baseline.json').write_text(json.dumps(result,indent=2))
