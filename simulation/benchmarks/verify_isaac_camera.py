"""Exercise actual viewport mouse routing and persistence using Kit's input queue."""
import sys,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'isaac'))
from isaacsim import SimulationApp
app=SimulationApp({'headless':False,'active_gpu':0,'multi_gpu':False,'width':1280,'height':800,'extra_args':['--/exts/omni.kit.viewport.window/startup/cameraManipulator/enabled=false']})
try:
    import carb.input
    import omni.appwindow
    import omni.ui as ui
    import omni.usd
    import numpy as np
    from pxr import UsdGeom,UsdShade
    from omni.kit.viewport.utility import get_active_viewport
    from common.navigation import Navigation
    from scene import build
    from kinematics import Kinematics
    from camera import OrbitCamera
    context=omni.usd.get_context();context.new_stage();app.update();stage=context.get_stage()
    root,op=build(stage,'fast','navigation');nav=Navigation();Kinematics(stage,root).apply(nav)
    orbit=OrbitCamera(op,nav.position,attach=True)
    for _ in range(15):app.update()
    get_active_viewport().camera_path='/World/Camera'
    for _ in range(5):app.update()
    print('Viewport camera surface',orbit.surface.screen_position_x,orbit.surface.screen_position_y,orbit.surface.computed_width,orbit.surface.computed_height,flush=True)
    mx=orbit.surface.screen_position_x+orbit.surface.computed_width*.5
    my=orbit.surface.screen_position_y+orbit.surface.computed_height*.5
    mouse=omni.appwindow.get_default_app_window().get_mouse();provider=carb.input.acquire_input_provider()
    width=ui.Workspace.get_main_window_width();height=ui.Workspace.get_main_window_height();dpi=ui.Workspace.get_dpi_scale()
    def event(kind,x=mx,y=my):
        provider.buffer_mouse_event(mouse,kind,(x*dpi/width,y*dpi/height),0,(x*dpi,y*dpi))
        for _ in range(3):app.update()
        orbit.update(nav.position)
    E=carb.input.MouseEventType
    before=np.array(op.Get())
    event(E.MOVE);event(E.LEFT_BUTTON_DOWN)
    for i in range(1,7):event(E.MOVE,mx+10*i,my+4*i)
    event(E.LEFT_BUTTON_UP,mx+60,my+24)
    rotated=np.array(op.Get());assert not np.allclose(before,rotated),'Mouse drag did not reach viewport overlay'
    assert not orbit.follow
    for _ in range(30):orbit.update(nav.position);app.update()
    assert np.allclose(np.array(op.Get()),rotated),'Manual view was overwritten'
    assert not carb.settings.get_settings().get('/exts/omni.kit.viewport.window/bindings/camera')
    event(E.RIGHT_BUTTON_DOWN,mx+60,my+24)
    pan_steps=[]
    camera_xform=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Camera'))
    previous=np.array(op.Get())
    for i in range(1,13):
        event(E.MOVE,mx+60+3*i,my+24+2*i)
        current=np.array(op.Get())
        pan_steps.append(float(np.linalg.norm(current[3,:3]-previous[3,:3])))
        # Check actual composed transform, including any ops a competing
        # native manipulator might append or overwrite between updates.
        for _ in range(3):
            app.update()
            assert np.allclose(np.array(camera_xform.GetLocalTransformation()),current), 'Camera changed outside custom controller'
        previous=current
    event(E.RIGHT_BUTTON_UP,mx+96,my+48)
    assert max(pan_steps)<.01,pan_steps
    assert max(pan_steps)-min(pan_steps)<1e-6,pan_steps
    # Simulate cursor re-centering while held; reject the discontinuity.
    orbit.press(100,100,1);previous=np.array(op.Get())
    orbit.move(1100,900);orbit.update(nav.position)
    assert np.allclose(np.array(op.Get()),previous)
    orbit.release(1100,900,1)
    panned=np.array(op.Get());assert not np.allclose(panned,rotated)
    orbit.wheel(0,1);orbit.update(nav.position);assert not np.allclose(np.array(op.Get()),panned)
    orbit.reset(nav.position);assert orbit.follow
    assert np.allclose(np.array(op.Get()),before)
    result=dict(passed=True,viewport_mouse_drag=True,manual_view_persists=True,right_drag_pan=True,stable_pan_steps=pan_steps,native_binding_disabled=True,coordinate_warp_filtered=True,zoom=True,reset=True)
    (ROOT/'outputs/isaac_camera_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    orbit.close()
finally:app.close()
