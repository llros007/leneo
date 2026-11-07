"""Check both completed tasks move the chassis forward without retreat or turning."""
from pathlib import Path
import json
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
result={}
for engine in ['mujoco','isaac']:
    d=json.loads((ROOT.parent/f'docs/validation/{engine}_straight_report.json').read_text())
    assert d['error'] is None and d['task']['success'] and d['task']['complete']
    points=np.asarray([d['start_position']]+[t['position'] for t in d['trajectory']]+[d['end_position']])
    quats=np.asarray([t['quaternion'] for t in d['trajectory']])
    yaw=np.arctan2(2*quats[:,0]*quats[:,3],1-2*quats[:,3]**2)
    heading_error=np.arctan2(np.sin(yaw-np.pi/2),np.cos(yaw-np.pi/2))
    assert np.ptp(points[:,0])<1e-10 and np.min(np.diff(points[:,1]))>=-1e-10
    assert np.max(np.abs(heading_error))<1e-10 and d['blocked_steps']==0
    travel=float(points[-1,1]-points[0,1]);assert .34<travel<.38
    phases=[e['phase'] for e in d['task']['events']]
    assert 'approach-bar' in phases and not set(phases)&{'reverse','left-aisle','along-aisle','face-bar'}
    if engine=='isaac':
        assert d['orange_physics']['render_pose_samples']>100
        assert d['orange_physics']['max_post_render_grip_error_m']<.002
    result[engine]=dict(passed=True,forward_travel_m=travel,max_heading_error_rad=float(np.max(np.abs(heading_error))),blocked_steps=d['blocked_steps'],sim_time=d['sim_time'],task_complete=True)
(ROOT/'outputs/straight_route_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
