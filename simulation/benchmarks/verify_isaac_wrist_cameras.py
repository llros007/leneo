"""Validate native USD wrist cameras after the normal URDF conversion."""
from pathlib import Path
import sys,json,ctypes,math
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'isaac'))
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':0,'multi_gpu':False,'width':320,'height':240})
try:
    import numpy as np
    import omni.usd
    from pxr import Usd,UsdGeom,Gf
    from PIL import Image
    from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_buffer
    from common.navigation import Navigation
    from scene import build
    from kinematics import Kinematics
    context=omni.usd.get_context();context.new_stage();app.update();stage=context.get_stage()
    robot,_=build(stage,'fast','navigation');nav=Navigation();kin=Kinematics(stage,robot);kin.apply(nav)
    config=json.loads((ROOT/'assets/leneo_description/config/wrist_cameras.json').read_text())
    cameras={}
    for p in Usd.PrimRange(robot):
        if p.IsA(UsdGeom.Camera):cameras[p.GetName()]=p
    assert set(cameras)==set(config['camera_names']),list(cameras)
    def positions():
        cache=UsdGeom.XformCache()
        return {n:np.array(cache.GetLocalToWorldTransform(p)) for n,p in cameras.items()}
    before=positions();nav.joints['left_wrist_roll']+=.5;kin.apply(nav);after=positions()
    assert not np.allclose(before['left_wrist_camera'],after['left_wrist_camera'])
    assert np.allclose(before['right_wrist_camera'],after['right_wrist_camera'])
    nav.joints['left_wrist_roll']-=.5;kin.apply(nav)
    q=config['camera_opengl_quat_wxyz']
    expected=Gf.Matrix4d().SetRotate(Gf.Quatd(q[0],Gf.Vec3d(*q[1:])))
    expected.SetTranslateOnly(Gf.Vec3d(*config['camera_position_in_gripper_m']))
    out=ROOT/'outputs/wrist_cameras';out.mkdir(parents=True,exist_ok=True)
    viewport=get_active_viewport();assert viewport is not None
    viewport.resolution=tuple(config['resolution'])
    pointer=ctypes.pythonapi.PyCapsule_GetPointer
    pointer.argtypes=[ctypes.py_object,ctypes.c_char_p];pointer.restype=ctypes.c_void_p
    paths={}
    for name,p in cameras.items():
        cam=UsdGeom.Camera(p)
        assert np.allclose(np.array(cam.GetLocalTransformation()),np.array(expected),atol=1e-9)
        fov=math.degrees(2*math.atan(cam.GetVerticalApertureAttr().Get()/(2*cam.GetFocalLengthAttr().Get())))
        assert abs(fov-config['fovy_deg'])<1e-4
        viewport.camera_path=str(p.GetPath());paths[name]=str(p.GetPath())
        for _ in range(35):app.update()
        received=[]
        def callback(buffer,size,width,height,byte_format):
            assert size==width*height*4
            pixels=np.frombuffer(ctypes.string_at(pointer(buffer,None),size),dtype=np.uint8).reshape(height,width,4).copy()
            received.append(pixels[:,:,:3])
        ticket=capture_viewport_to_buffer(viewport,callback)
        for _ in range(120):
            app.update()
            if received:break
        assert received,'Native camera rendering timed out'
        pixels=received[0];assert pixels.shape==(240,320,3) and pixels.std()>5
        Image.fromarray(pixels).save(out/('isaac_'+name+'.png'))
    report=dict(passed=True,cameras=paths,resolution=config['resolution'],fovy_deg=config['fovy_deg'],independent_wrist_motion=True,native_rtx_capture=True)
    (out/'isaac_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
finally:app.close()
