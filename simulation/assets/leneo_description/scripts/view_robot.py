import pathlib,sys,time,argparse
R=pathlib.Path(__file__).resolve().parents[1]
try:import mujoco
except ImportError:
 # Local development fallback; portable use installs requirements-simulation.txt.
 candidate=next((p/'simulation/leneo_description/_validation_deps' for p in R.parents if (p/'simulation/leneo_description/_validation_deps').exists()),None)
 if candidate:sys.path.insert(0,str(candidate))
 import mujoco
def main():
 p=argparse.ArgumentParser();p.add_argument('--headless',action='store_true');p.add_argument('--viewer-seconds',type=float);args=p.parse_args()
 m=mujoco.MjModel.from_xml_path(str(R/'mujoco/leneo_robot_only.xml'));d=mujoco.MjData(m)
 mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
 if args.headless:
  for _ in range(200):mujoco.mj_step(m,d)
  print(f'Robot-only OK: {m.nbody} bodies, {m.nu} actuators; no room, no floor; gravity=0');return
 from mujoco import viewer as mjviewer
 with mjviewer.launch_passive(m,d) as v:
  with v.lock():
   v.opt.geomgroup[0]=0;v.opt.geomgroup[3]=0
   v.cam.lookat[:]=[0,0,.45];v.cam.distance=1.9;v.cam.azimuth=-55;v.cam.elevation=-20
  start=time.monotonic()
  while v.is_running():
   if args.viewer_seconds is not None and time.monotonic()-start>=args.viewer_seconds:break
   # Pose inspection stays at home; native viewer remains interactive.
   mujoco.mj_forward(m,d);v.sync();time.sleep(.015)
if __name__=='__main__':main()
