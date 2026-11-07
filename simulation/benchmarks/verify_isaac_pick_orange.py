"""Run/validate native Isaac Sim 6.2 grasping, gravity release and camera cues."""
import argparse,json,math,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--reuse-report',action='store_true');p.add_argument('--windowed',action='store_true');p.add_argument('--report',type=Path);p.add_argument('--captures',type=Path);args=p.parse_args()
report=args.report or ROOT/'outputs/isaac_pick_orange_report.json';captures=args.captures or ROOT/'outputs/isaac_orange_stages'
if not args.reuse_report:
    with (ROOT/'outputs/isaac_pick_orange_verification.log').open('w') as log:
        command=[str(ROOT/'isaac/run.sh'),'--demo-pick-orange','--report',str(report),'--capture-stages',str(captures)]
        command+=['--seconds','62'] if args.windowed else ['--headless']
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
d=json.loads(report.read_text());task=d['task'];physics=d['orange_physics']
assert d['error'] is None and task['success'] and task['complete'],d
assert task['released'] and not task['attached'] and not physics['kinematic_enabled']
assert physics['grasp_samples']>=30 and physics['max_jaw_surface_gap_m']<.004
assert task['grasp_geometry']['between_fingers']
assert physics.get('render_pose_samples',0)>0, 'Grasp was never checked after rendering'
assert physics['max_render_pose_error_m']<.002 and physics['max_post_render_grip_error_m']<.002
assert physics['release_height']-physics['lowest_after_release']>.005,'No physical fall after release'
assert math.sqrt(sum(v*v for v in physics['orange_velocity']))<.01
assert abs(physics['orange_position'][2]-(task['table_height']+.02219412875))<.012
assert [e['view'] for e in d['camera_events']]==['pick','default','place']
assert d['blocked_steps']==0 and d['render_settings']['light_scale']==1.6
for name in ['pick','carrying','above_bar','released','final']:assert (captures/(name+'.png')).stat().st_size>10000
result=dict(render_pose_samples=physics['render_pose_samples'],max_post_render_grip_error_m=physics['max_post_render_grip_error_m'],passed=True,simulation_seconds=d['sim_time'],native_physics=physics['engine'],max_jaw_surface_gap_mm=physics['max_jaw_surface_gap_m']*1000,
            release_fall_mm=(physics['release_height']-physics['lowest_after_release'])*1000,final_velocity_mps=physics['orange_velocity'],camera_events=d['camera_events'],light_scale=d['render_settings']['light_scale'])
(ROOT/'outputs/isaac_orange_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
