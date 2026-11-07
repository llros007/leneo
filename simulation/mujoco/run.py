"""LeNeo kitchen: fast keyboard navigation or optional wheel-force dynamics."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
from scipy.optimize import least_squares
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.control import Control, ROOT, HELP
from common.navigation import Navigation
from common.orange_layout import NAMES
from build_scene import build
from task_scene import SPAWN


class Simulation:
    def __init__(self,quality='fast',drive_mode='navigation'):
        import mujoco
        self.mj=mujoco;self.mode=drive_mode
        start=time.perf_counter();self.model=mujoco.MjModel.from_xml_path(str(build(quality)));self.load_seconds=time.perf_counter()-start
        self.data=mujoco.MjData(self.model);mujoco.mj_resetDataKeyframe(self.model,self.data,0)
        self.dt=.01 if self.mode=='navigation' else .002
        self.substeps=5 if self.mode=='navigation' else 1
        self.model.opt.timestep=self.dt/self.substeps
        self.navigation=Navigation(position=SPAWN) if self.mode=='navigation' else None
        self.joints={self.model.joint(i).name:int(self.model.jnt_qposadr[i]) for i in range(self.model.njnt)}
        self.actuator_joints=[self.model.joint(int(self.model.actuator_trnid[i,0])).name for i in range(self.model.nu)]
        self.rates=np.zeros(2)
        self.orange_q=int(self.model.joint('orange_free').qposadr[0])
        self.orange_v=int(self.model.joint('orange_free').dofadr[0])
        self.grip_site=self.model.site('orange_grip').id
        self.grasp_eq=self.model.equality('orange_grasp').id
        self.anchor=self.model.body('grasp_anchor').mocapid[0]
        from grasp_geometry import finger_meshes
        self.finger_surfaces=finger_meshes(self.model)
        self.grasp_geometry=None
        self.attached=False;self.grasp_offset=np.zeros(3);self.grasp_rotation=np.eye(3)
        if self.navigation:
            # Prescribed robot pose; only the fruit has dynamic contacts.
            self.model.body_gravcomp[:]=1
            self.model.body_gravcomp[self.model.body('orange').id]=0
            for i in range(self.model.ngeom):
                if self.model.geom_bodyid[i] not in [0,self.model.body('orange').id]:
                    self.model.geom_contype[i]=self.model.geom_conaffinity[i]=0
        mujoco.mj_forward(self.model,self.data)

    @property
    def time(self):return float(self.data.time)

    def orange_position(self):return self.data.qpos[self.orange_q:self.orange_q+3].copy()

    def orange_velocity(self):return self.data.qvel[self.orange_v:self.orange_v+3].copy()

    def solve_ik(self,target):
        s=self;m=s.model;d=s.mj.MjData(m);s.mj.mj_copyData(d,m,s.data)
        adr=[s.joints[n] for n in NAMES];bounds=np.array([m.joint(n).range for n in NAMES])
        seed=s.data.qpos[adr].copy();best=None
        def fun(q):
            d.qpos[adr]=q;s.mj.mj_forward(m,d)
            # Point the fingers down; yaw remains free to use the five-axis arm.
            axis=d.site_xmat[s.grip_site].reshape(3,3)[:,2]
            position=d.site_xpos[s.grip_site].copy()
            if s.attached:position+=d.site_xmat[s.grip_site].reshape(3,3)@s.grasp_offset
            return np.r_[(position-target)*10,(axis-[0,0,-1])*.12]
        seeds=[seed, np.array([-.20,0,.2,1.3,-.4,0]),np.array([.30,0,-.8,1.3,-1,0])]
        for q in seeds:
            result=least_squares(fun,np.clip(q,bounds[:,0]+1e-6,bounds[:,1]-1e-6),bounds=(bounds[:,0]+1e-7,bounds[:,1]-1e-7),max_nfev=250,ftol=1e-8,xtol=1e-8,gtol=1e-8)
            err=np.linalg.norm(fun(result.x)[:3])/10
            if best is None or err<best[0]:best=(err,result.x)
            if err<.001:break
        return dict(zip(NAMES,best[1])),float(best[0])


    def grasp(self,active):
        if active:
            delta=self.data.qpos[self.orange_q:self.orange_q+3]-self.data.site_xpos[self.grip_site]
            if np.linalg.norm(delta)>.025:raise RuntimeError(f'Fruit outside gripper: {delta}')
            from grasp_geometry import measure
            self.grasp_geometry=measure(self)
            if not self.grasp_geometry['between_fingers']:raise RuntimeError(f'Orange is not between both fingers: {self.grasp_geometry}')
            R=self.data.site_xmat[self.grip_site].reshape(3,3)
            self.grasp_offset=R.T@delta
            self.grasp_rotation=R.T@self.data.xmat[self.model.body('orange').id].reshape(3,3)
            self.data.mocap_pos[self.anchor]=self.data.qpos[self.orange_q:self.orange_q+3]
            self.data.mocap_quat[self.anchor]=self.data.qpos[self.orange_q+3:self.orange_q+7]
        self.data.eq_active[self.grasp_eq]=int(active);self.attached=active

    def step(self,control):
        d,m=self.data,self.model
        if self.navigation:
            self.navigation.step(control,self.dt)
            d.qpos[:3]=self.navigation.position;d.qpos[3:7]=self.navigation.quaternion
            for name,value in self.navigation.joints.items():d.qpos[self.joints[name]]=value
            d.qvel[:self.orange_v]=0;self.mj.mj_forward(m,d)
            robot_pose=d.qpos[:self.orange_q].copy()
            if self.attached:
                R=d.site_xmat[self.grip_site].reshape(3,3)
                d.mocap_pos[self.anchor]=d.site_xpos[self.grip_site]+R@self.grasp_offset
                self.mj.mju_mat2Quat(d.mocap_quat[self.anchor],(R@self.grasp_rotation).flatten())
            for _ in range(self.substeps):
                self.mj.mj_step(m,d)
                d.qpos[:self.orange_q]=robot_pose;d.qvel[:self.orange_v]=0
                self.mj.mj_forward(m,d)
        else:
            self.rates+=np.clip(np.asarray(control.wheels())-self.rates,-12*self.dt,12*self.dt)
            for i,joint in enumerate(self.actuator_joints):
                if joint in ['left_wheel_joint','right_wheel_joint']:target=self.rates[0 if joint.startswith('left') else 1]
                else:
                    speed=.05 if joint=='lift_joint' else 2
                    target=d.ctrl[i]+np.clip(control.targets[joint]-d.ctrl[i],-speed*self.dt,speed*self.dt)
                d.ctrl[i]=target
            self.mj.mj_step(m,d)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true');p.add_argument('--steps',type=int,default=0);p.add_argument('--seconds',type=float)
    p.add_argument('--demo-pick-orange',action='store_true',help='IK + assisted-grasp floor-to-counter demonstration');p.add_argument('--demo',action='store_true');p.add_argument('--command-file');p.add_argument('--quality',choices=['fast','full'],default='fast')
    p.add_argument('--drive-mode',choices=['navigation','physics'],default='navigation')
    p.add_argument('--fps',type=float,default=60);p.add_argument('--width',type=int,default=960);p.add_argument('--height',type=int,default=640)
    p.add_argument('--screenshot',type=Path);p.add_argument('--report',type=Path,default=ROOT/'outputs/mujoco_report.json')
    args=p.parse_args()
    if args.steps<0 or args.fps<=0 or (args.seconds is not None and args.seconds<=0):p.error('steps >= 0, fps/seconds > 0 required')
    if args.demo_pick_orange and (args.demo or args.command_file):p.error('pick-orange cannot be combined with --demo or --command-file')
    if args.headless:os.environ.setdefault('MUJOCO_GL','egl')
    sim=Simulation(args.quality,args.drive_mode);model,data=sim.model,sim.data;mujoco=sim.mj
    limit=int(round(args.seconds/sim.dt)) if args.seconds else args.steps
    if args.headless and not limit:limit=round(5/sim.dt)
    control=Control();window=None
    task=None
    if args.demo_pick_orange:
        if not sim.navigation:p.error('--demo-pick-orange requires --drive-mode navigation')
        from pick_orange import PickOrange
        task=PickOrange(sim,control)
        if args.headless and not args.seconds and not args.steps:limit=round(150/sim.dt)
    if not args.headless:
        from window import Window
        window=Window(model,control,args.width,args.height,args.quality)
    director=None
    if task:
        from demo_camera import DemoCamera,camera_default
        director=DemoCamera(window.camera if window else camera_default())
        if window:window.demo_camera=director
    print(f'{HELP}\nMode: {args.drive_mode} | quality: {args.quality}',flush=True)
    count=0;error=None;frames=0;step_time=render_time=0.0;trace=[];loop_wall=None
    start=data.qpos[:3].copy();wall_start=last=time.perf_counter();accumulator=0.0
    try:
        while not control.quit and (not limit or count<limit):
            frame_start=time.perf_counter()
            if window and not window.poll():break
            if window:
                accumulator+=min(frame_start-last,.1);last=frame_start
                steps=min(int((accumulator+1e-9)/sim.dt),round(.1/sim.dt))
            else:steps=min(50,limit-count)
            for _ in range(steps):
                if limit and count>=limit:break
                control.tick(sim.dt);control.read_command(args.command_file)
                if args.demo:control.demo(count*sim.dt)
                if task:task.tick()
                t=time.perf_counter();sim.step(control);step_time+=time.perf_counter()-t;count+=1
                if director:
                    changed=director.update(task.phase,float(data.time))
                    if changed and window:window.follow=False
                if window:accumulator=max(0,accumulator-sim.dt)
                if task and task.done and args.headless and not args.seconds and not args.steps:control.quit=True;break
                if count%max(1,round(.5/sim.dt))==0:
                    trace.append(dict(time=float(data.time),position=data.qpos[:3].tolist(),quaternion=data.qpos[3:7].tolist()))
                    if not np.isfinite(data.qpos).all() or abs(data.qpos[2])>2:raise RuntimeError('Simulation became unstable')
            if window:
                t=time.perf_counter();window.render(data,f'{args.drive_mode} | {frames/max(.001,t-wall_start):.0f} FPS | t={data.time:.1f}s'+(' | obstacle: STOP' if sim.navigation and sim.navigation.blocked else '')+(' | '+task.phase if task else ''))
                render_time+=time.perf_counter()-t;frames+=1
                time.sleep(max(0,1/args.fps-(time.perf_counter()-frame_start)))
        loop_wall=time.perf_counter()-wall_start
        if args.screenshot:
            args.screenshot.parent.mkdir(parents=True,exist_ok=True)
            from window import camera_default
            camera=window.camera if window else (director.camera if director else camera_default())
            option=mujoco.MjvOption();option.geomgroup[0]=option.geomgroup[3]=0
            with mujoco.Renderer(model,height=args.height,width=args.width) as renderer:
                renderer.update_scene(data,camera=camera,scene_option=option)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=args.quality=='full'
                from PIL import Image
                Image.fromarray(renderer.render()).save(args.screenshot)
    except Exception as exc:error=str(exc);raise
    finally:
        report=dict(camera_events=director.events if director else [],task=task.report() if task else None,engine='mujoco',version=mujoco.__version__,drive_mode=args.drive_mode,quality=args.quality,steps=count,sim_time=float(data.time),
            load_seconds=sim.load_seconds,wall_seconds=loop_wall if loop_wall is not None else time.perf_counter()-wall_start,frames=frames,step_ms=1000*step_time/max(1,count),render_ms=1000*render_time/max(1,frames),
            unique_triangles=int(sum(model.mesh_facenum)),instanced_triangles=sum(int(model.mesh_facenum[model.geom_dataid[i]]) for i in range(model.ngeom) if model.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH),
            start_position=start.tolist(),end_position=data.qpos[:3].tolist(),trajectory=trace,
            joint_positions={n:float(data.qpos[sim.joints[n]]) for n in ['lift_joint','left_shoulder_pan','right_shoulder_pan']},
            blocked_steps=sim.navigation.blocked_steps if sim.navigation else 0,finite=bool(np.isfinite(data.qpos).all()),warnings={str(i):int(w.number) for i,w in enumerate(data.warning) if w.number},error=error)
        args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='trajectory'}),flush=True)
        if window:window.close()

if __name__=='__main__':main()
