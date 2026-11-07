"""Shared stage setup for navigation and optional force-based physics."""
import json
from pxr import UsdGeom,UsdPhysics,UsdLux,UsdShade,PhysxSchema,Gf
from common.control import ROOT
from common.navigation import SPAWN,SPAWN_YAW


def build(stage,quality,mode,light_scale=1.6):
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1)
    UsdGeom.Xform.Define(stage,'/World')
    kitchen=stage.DefinePrim('/World/Kitchen','Xform');kitchen.GetReferences().AddReference(str(ROOT/'assets/modern_kitchen/kitchen.usdc'))
    if mode=='physics':
        scene=UsdPhysics.Scene.Define(stage,'/World/Physics');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
        physics=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());physics.CreateEnableGPUDynamicsAttr(False);physics.CreateBroadphaseTypeAttr('MBP')
        material=UsdPhysics.MaterialAPI.Apply(stage.DefinePrim('/World/Friction','Material'))
        material.CreateStaticFrictionAttr(.9);material.CreateDynamicFrictionAttr(.8);material.CreateRestitutionAttr(0)
        def collider(path,center,half):
            cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(2);cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3f(*half))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim());api=PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim());api.CreateContactOffsetAttr(.001);api.CreateRestOffsetAttr(0)
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(UsdShade.Material(material.GetPrim()),materialPurpose='physics');cube.CreateVisibilityAttr('invisible')
        collider('/World/Ground',[0,0,-.05],[10,10,.05])
        boxes=json.loads((ROOT/'assets/modern_kitchen/manifest.json').read_text())['collision_boxes']
        for i,b in enumerate(boxes):collider(f'/World/Collision/box_{i}',b['center'],b['half'])
    light=UsdLux.DomeLight.Define(stage,'/World/Sky');light.CreateIntensityAttr(950*light_scale);light.CreateColorAttr(Gf.Vec3f(.9,.93,1))
    light=UsdLux.DistantLight.Define(stage,'/World/Sun');light.CreateIntensityAttr(1800*light_scale);light.AddRotateXYZOp().Set(Gf.Vec3f(315,0,40))
    light=UsdLux.RectLight.Define(stage,'/World/CeilingSoftbox');light.CreateIntensityAttr(800*light_scale);light.CreateWidthAttr(3);light.CreateHeightAttr(2);light.AddTranslateOp().Set(Gf.Vec3d(-.5,1,2.85))
    # Interior broad sources light the aisle without washing black polymer grey.
    fill=UsdLux.RectLight.Define(stage,'/World/AisleFill');fill.CreateIntensityAttr(500*light_scale);fill.CreateWidthAttr(2.5);fill.CreateHeightAttr(1.5)
    fill.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(.3,-.8,2.3),Gf.Vec3d(-1.2,.5,.65),Gf.Vec3d(0,0,1)).GetInverse())
    folder=ROOT/('assets/isaac_robot_fast' if quality=='fast' else 'assets/isaac_robot')
    path=(folder/'robot_path.txt').read_text().strip()
    robot=stage.DefinePrim('/World/Robot','Xform');robot.GetReferences().AddReference(path)
    robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('none' if mode=='navigation' else 'physx')
    import math
    x=UsdGeom.Xformable(robot);x.AddTranslateOp().Set(Gf.Vec3d(*SPAWN));x.AddOrientOp().Set(Gf.Quatf(math.cos(SPAWN_YAW/2),Gf.Vec3f(0,0,math.sin(SPAWN_YAW/2))))
    from appearance import apply
    apply(stage,robot)
    camera=UsdGeom.Camera.Define(stage,'/World/Camera');camera.CreateFocalLengthAttr(18);camera.CreateClippingRangeAttr(Gf.Vec2f(.01,100))
    camera_op=camera.AddTransformOp()
    return robot,camera_op


def follow_camera(op,position):
    target=Gf.Vec3d(*map(float,position))+Gf.Vec3d(0,0,.5)
    eye=target+Gf.Vec3d(.9,.8,.8)
    op.Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
