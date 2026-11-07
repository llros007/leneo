"""LeNeo kitchen with independent fixed-rate navigation/physics and RTX rendering."""
import argparse
import json
from pathlib import Path
import sys
import time
import math
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.control import Control,ROOT,HELP
from common.navigation import Navigation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true');p.add_argument('--steps',type=int,default=0);p.add_argument('--seconds',type=float)
    p.add_argument('--quality',choices=['fast','full'],default='fast');p.add_argument('--drive-mode',choices=['navigation','physics'],default='navigation')
    p.add_argument('--renderer',choices=['RaytracedLighting','PathTracing'],default='RaytracedLighting')
    p.add_argument('--fps',type=float,default=60);p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=800)
    p.add_argument('--demo-pick-orange',action='store_true');p.add_argument('--light-scale',type=float,default=1.6,help='light intensity multiplier (previous brightness: 1.0)');p.add_argument('--demo',action='store_true');p.add_argument('--command-file');p.add_argument('--benchmark-frames',type=int,default=0)
    p.add_argument('--video',type=Path);p.add_argument('--video-fps',type=int,default=30)
    p.add_argument('--capture-stages',type=Path,help='save demo camera milestones');p.add_argument('--screenshot',type=Path);p.add_argument('--report',type=Path,default=ROOT/'outputs/isaac_report.json')
    args=p.parse_args()
    if args.steps<0 or args.fps<=0 or (args.seconds is not None and args.seconds<=0):p.error('steps >= 0, fps/seconds > 0 required')
    if args.video and not args.headless:p.error('--video uses deterministic recording; add --headless')
    if args.video_fps<1 or args.video_fps>100:p.error('--video-fps must be between 1 and 100')
    if args.light_scale<=0:p.error('--light-scale must be positive')
    if args.demo_pick_orange and (args.drive_mode!='navigation' or args.demo or args.command_file):p.error('pick-orange requires navigation and cannot combine with --demo/--command-file')
    from isaacsim import SimulationApp
    load_start=time.perf_counter()
    app=SimulationApp({'headless':args.headless,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'width':args.width,'height':args.height,
        'extra_args':['--/exts/omni.kit.viewport.window/startup/cameraManipulator/enabled=false'],
        'sync_loads':True,'renderer':args.renderer,'samples_per_pixel_per_frame':1 if args.quality=='fast' else 4})
    count=frames=0;step_time=render_time=0.0;error=None;subscription=None;trace=[];orbit=None;task=None;orange=None;director=None;video=None
    report=dict(engine='isaacsim',version='6.2.0',drive_mode=args.drive_mode,quality=args.quality,renderer=args.renderer)
    try:
        import numpy as np
        import omni.usd
        import omni.timeline
        import carb
        from pxr import UsdGeom
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        from scene import build
        from camera import OrbitCamera
        settings=carb.settings.get_settings();settings.set('/persistent/app/viewport/displayOptions',0)
        settings.set('/rtx/post/motionblur/enabled',False)
        settings.set('/rtx/post/aa/op',4)  # native-resolution DLAA
        settings.set('/rtx-transient/dlssg/enabled',False)
        context=omni.usd.get_context();context.new_stage();app.update();stage=context.get_stage()
        root,camera_op=build(stage,args.quality,args.drive_mode,args.light_scale)
        timeline=omni.timeline.get_timeline_interface();timeline.stop();app.update()
        control=Control()
        from common.orange_layout import SPAWN as ORANGE_SPAWN
        nav=Navigation(position=ORANGE_SPAWN) if args.demo_pick_orange else (Navigation() if args.drive_mode=='navigation' else None)
        if nav:
            from kinematics import Kinematics
            kinematics=Kinematics(stage,root);kinematics.apply(nav);dt=.01
            state=lambda:(nav.position,nav.quaternion,nav.joints)
        else:
            from physics import Physics
            physics=Physics(root,app,timeline);dt=physics.dt;state=physics.state
        if args.demo_pick_orange:
            from orange_demo import OrangeDemo
            from common.pick_orange import PickOrange
            orange=OrangeDemo(stage,nav,app,timeline);task=PickOrange(orange,control)
        position,rotation,joints=state();start=np.asarray(position).copy()
        orbit=OrbitCamera(camera_op,position,attach=not args.headless)
        if task:
            from common.demo_camera import DemoCamera,camera_default,recording_default
            director=DemoCamera(camera_default(),overrides={'default':recording_default()} if args.video else None);orbit.on_manual=director.manual
            orbit.apply_demo_pose(director.camera);orbit.update(position)
            cam=UsdGeom.Camera(stage.GetPrimAtPath('/World/Camera'))
            cam.CreateVerticalApertureAttr(14.911688);cam.CreateHorizontalApertureAttr(14.911688*args.width/args.height)
        app.update();get_active_viewport().camera_path='/World/Camera'
        get_active_viewport().resolution=(args.width,args.height)
        # Stage/viewport creation restores persisted RTX settings; set AA after it.
        settings.set('/rtx/post/aa/op',4)
        settings.set('/rtx-transient/dlssg/enabled',False)
        if not args.headless:
            import carb.input
            import omni.appwindow
            input_api=carb.input.acquire_input_interface();app_window=omni.appwindow.get_default_app_window();keyboard=app_window.get_keyboard()
            def key(event,*_):
                if event.type==carb.input.KeyboardEventType.KEY_PRESS:
                    if event.input.name=='V':orbit.toggle_follow()
                    elif event.input.name=='B':orbit.reset(state()[0])
                    else:control.press(event.input.name)
                elif event.type==carb.input.KeyboardEventType.KEY_RELEASE:control.release(event.input.name)
                return True
            subscription=input_api.subscribe_to_keyboard_events(keyboard,key)
        limit=round(args.seconds/dt) if args.seconds else args.steps
        if args.headless and not limit:limit=round((150 if task else 5)/dt)
        report['load_seconds']=time.perf_counter()-load_start
        report['render_settings']=dict(light_scale=args.light_scale,aa_mode=settings.get('/rtx/post/aa/op'),width=args.width,height=args.height)
        if args.video:
            from video import ViewportVideo
            video=ViewportVideo(args.video,args.width,args.height,args.video_fps,app,orbit,orange)
            video.capture(0.,task.phase if task else 'manual')
        print(HELP+f'\nMode: {args.drive_mode} | quality: {args.quality}',flush=True)
        wall_start=last=time.perf_counter();accumulator=0.0;simtime=0.0;captured=set()
        while app.is_running() and not control.quit and (not limit or count<limit):
            frame_start=time.perf_counter()
            if not args.headless and not app_window.is_focused():control.clear_keys();orbit.buttons.clear()
            if args.headless:
                steps=min(50,limit-count)
                if video:steps=min(steps,max(1,math.ceil((video.next_time-simtime-1e-9)/dt)))
            else:
                accumulator+=min(frame_start-last,.1);last=frame_start
                steps=min(int((accumulator+1e-9)/dt),round(.1/dt))
            camera_dirty=False
            for _ in range(steps):
                if limit and count>=limit:break
                control.tick(dt);control.read_command(args.command_file)
                if args.demo:control.demo(simtime)
                t=time.perf_counter()
                if task:
                    task.tick();orange.step(control)
                elif nav:nav.step(control,dt)
                else:physics.step(control)
                step_time+=time.perf_counter()-t;count+=1;simtime=count*dt
                if director:
                    blending=director.transition is not None
                    changed=director.update(task.phase,simtime)
                    camera_dirty|=changed or blending or director.transition is not None
                if task and task.done and args.headless and not args.seconds and not args.steps:control.quit=True;break
                if not args.headless:accumulator=max(0,accumulator-dt)
                if count%max(1,round(.5/dt))==0:
                    position,rotation,joints=state()
                    trace.append(dict(time=simtime,position=list(map(float,position)),quaternion=list(map(float,rotation))))
                    if not np.isfinite(position).all():raise RuntimeError('Non-finite robot position')
            if nav:kinematics.apply(nav)
            if camera_dirty:orbit.apply_demo_pose(director.camera)
            if video and simtime+1e-9>=video.next_time:
                orbit.update(state()[0]);video.capture(simtime,task.phase if task else 'manual')
            if args.capture_stages and task:
                label={'close':'pick','approach-bar':'carrying','place':'above_bar','open':'released','complete':'final'}.get(task.phase)
                if label and label not in captured and director.transition is None:
                    orbit.update(state()[0])
                    for _ in range(10):
                        app.update()
                        if orange:orange.verify_render_pose()
                    args.capture_stages.mkdir(parents=True,exist_ok=True)
                    capture_viewport_to_file(get_active_viewport(),str((args.capture_stages/(label+'.png')).resolve()))
                    for _ in range(10):
                        app.update()
                        if orange:orange.verify_render_pose()
                    captured.add(label)
            if control.quit and task and task.done:break
            if not args.headless:
                position,_,_=state();orbit.update(position)
                if task and orbit.status is not None:orbit.status.text=f'Pick orange | {task.phase} | {simtime:.1f}s'
                t=time.perf_counter();app.update();render_time+=time.perf_counter()-t;frames+=1
                if orange and app.is_running():orange.verify_render_pose()
                time.sleep(max(0,1/args.fps-(time.perf_counter()-frame_start)))
        position,rotation,joints=state()
        report.update(task=task.report() if task else None,orange_physics=orange.diagnostics() if orange else None,camera_events=director.events if director else [],steps=count,sim_time=count*dt,wall_seconds=time.perf_counter()-wall_start,frames=frames,
            step_ms=1000*step_time/max(1,count),render_ms=1000*render_time/max(1,frames),trajectory=trace,start_position=start.tolist(),
            end_position=list(map(float,position)),orientation=list(map(float,rotation)),
            joint_positions={n:float(joints[n]) for n in ['lift_joint','left_shoulder_pan','right_shoulder_pan']},
            blocked_steps=nav.blocked_steps if nav else 0,finite=bool(np.isfinite(position).all() and all(np.isfinite(v) for v in joints.values())))
        if args.screenshot or args.benchmark_frames:
            timeline.pause();app.update();get_active_viewport().camera_path='/World/Camera'
            orbit.update(position)
            for _ in range(10):app.update()
            t=time.perf_counter()
            for _ in range(args.benchmark_frames):app.update()
            if args.benchmark_frames:report['benchmark_render_ms']=1000*(time.perf_counter()-t)/args.benchmark_frames
            if args.screenshot:
                args.screenshot.parent.mkdir(parents=True,exist_ok=True)
                capture_viewport_to_file(get_active_viewport(),str(args.screenshot.resolve()))
                for _ in range(15):app.update()
        stage.GetRootLayer().Export(str(ROOT/'isaac/scene.usda'));timeline.stop()
    except Exception as exc:error=str(exc);raise
    finally:
        if video:
            video.close();report['video']=video.report()
        report['error']=error;args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='trajectory'}),flush=True)
        if subscription is not None:input_api.unsubscribe_to_keyboard_events(keyboard,subscription)
        if orbit is not None:orbit.close()
        app.close()

if __name__=='__main__':main()
