"""Native PhysX orange with URDF IK and explicit, geometry-gated grasp assistance."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf
from common.control import ROOT
from common.orange_layout import ORANGE,TABLE_TOP
from common.robot_ik import RobotIK


class OrangeDemo:
    dt=.01
    def __init__(self,stage,navigation,app,timeline):
        from isaacsim.core.experimental.prims import RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        self.stage=stage;self.navigation=navigation;self.ik=RobotIK(navigation);self.time=0.
        self.attached=False;self.grasp_geometry=None;self.grasp_offset=np.zeros(3);self.grasp_rotation=np.eye(3)
        info=json.loads((ROOT/'assets/orange/manifest.json').read_text());self.radii=np.asarray(info['dimensions'])/2
        scene=UsdPhysics.Scene.Define(stage,'/World/OrangePhysics');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
        api=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP')
        material=UsdPhysics.MaterialAPI.Apply(stage.DefinePrim('/World/OrangeFriction','Material'))
        material.CreateStaticFrictionAttr(.9);material.CreateDynamicFrictionAttr(.8);material.CreateRestitutionAttr(0.)
        def collider(path,position,half):
            cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(2);cube.AddTranslateOp().Set(Gf.Vec3d(*position));cube.AddScaleOp().Set(Gf.Vec3f(*half))
            cube.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            p=PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim());p.CreateContactOffsetAttr(.0005);p.CreateRestOffsetAttr(0.)
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(UsdShade.Material(material.GetPrim()),materialPurpose='physics')
        collider('/World/OrangeGround',[0,0,-.05],[10,10,.05])
        collider('/World/OrangeBar',[-.137662653,1.274041213,.886960607],[.897585054,.446533254,.021079949])
        root=stage.DefinePrim('/World/Orange','Xform');root.GetReferences().AddReference(str(ROOT/'assets/orange/Orange001/Orange001.usd'))
        child=stage.GetPrimAtPath('/World/Orange/Orange001')
        child.RemoveAPI(UsdPhysics.RigidBodyAPI);child.RemoveAPI(UsdPhysics.MassAPI)
        x=UsdGeom.Xformable(child);x.ClearXformOpOrder();x.AddTranslateOp().Set(Gf.Vec3d(0,0,-info['half_height']))
        x=UsdGeom.Xformable(root);x.ClearXformOpOrder();x.AddTranslateOp().Set(Gf.Vec3d(*ORANGE));x.AddOrientOp().Set(Gf.Quatf(1))
        self.api=UsdPhysics.RigidBodyAPI.Apply(root);self.api.CreateKinematicEnabledAttr(False)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(.09)
        PhysxSchema.PhysxRigidBodyAPI.Apply(root).CreateAngularDampingAttr(.7)
        self.collider=stage.GetPrimAtPath('/World/Orange/Orange001/Collisions/Orange001_C')
        UsdPhysics.MeshCollisionAPI.Apply(self.collider).CreateApproximationAttr('convexHull')
        contact=PhysxSchema.PhysxCollisionAPI.Apply(self.collider);contact.CreateContactOffsetAttr(.0005);contact.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(self.collider).Bind(UsdShade.Material(material.GetPrim()),materialPurpose='physics')
        SimulationManager.set_physics_sim_device('cpu');SimulationManager.set_physics_dt(.002)
        self.manager=SimulationManager;self.body=RigidPrim('/World/Orange')
        timeline.play();app.update();timeline.set_auto_update(False)
        self.body.set_world_poses(positions=[ORANGE.tolist()],orientations=[[1,0,0,0]])
        self.body.set_velocities([[0,0,0]],[[0,0,0]])
        self.max_render_pose_error=0.;self.max_post_render_grip_error=0.;self.render_pose_samples=0
        self.max_grasp_gap=0.;self.grasp_samples=0;self.release_height=None;self.lowest_after_release=None

    def pose(self):
        p,q=self.body.get_world_poses();return p.numpy()[0].astype(float),q.numpy()[0].astype(float)

    def orange_position(self):return self.pose()[0]

    def orange_velocity(self):return self.body.get_velocities()[0].numpy()[0].astype(float)

    def solve_ik(self,target):return self.ik.solve(target,self.attached,self.grasp_offset)

    def grasp(self,active):
        if active:
            center,q=self.pose();Rfruit=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
            self.grasp_geometry=self.ik.measure(center,Rfruit,self.radii)
            if not self.grasp_geometry['between_fingers']:raise RuntimeError(f'Orange is not between both fingers: {self.grasp_geometry}')
            p,R=self.ik.grip();self.grasp_offset=R.T@(center-p);self.grasp_rotation=R.T@Rfruit
        else:
            self.release_height=float(self.orange_position()[2]);self.lowest_after_release=self.release_height
        if active:self.body.set_velocities([[0,0,0]],[[0,0,0]])
        self.api.GetKinematicEnabledAttr().Set(bool(active));self.attached=active
        if not active:self.body.set_velocities([[0,0,0]],[[0,0,0]])

    def step(self,control):
        self.navigation.step(control,self.dt)
        if self.attached:
            p,R=self.ik.grip();q=Rotation.from_matrix(R@self.grasp_rotation).as_quat()
            position=p+R@self.grasp_offset;wxyz=q[[3,0,1,2]]
            # PhysX tensor teleports alone do not update the authored kinematic
            # transform used by USD/RTX; app.update can restore that stale pose.
            prim=self.stage.GetPrimAtPath('/World/Orange')
            prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*position))
            prim.GetAttribute('xformOp:orient').Set(Gf.Quatf(float(wxyz[0]),Gf.Vec3f(*wxyz[1:])))
            self.body.set_world_poses(positions=[position.tolist()],orientations=[wxyz.tolist()])
        self.manager.step(steps=5);self.time+=self.dt
        if self.release_height is not None:self.lowest_after_release=min(self.lowest_after_release,float(self.orange_position()[2]))
        if self.attached and round(self.time/self.dt)%100==0:
            center,q=self.pose();R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();geometry=self.ik.measure(center,R,self.radii)
            if not geometry['between_fingers']:raise RuntimeError(f'Orange left the fingers: {geometry}')
            self.grasp_samples+=1;self.max_grasp_gap=max(self.max_grasp_gap,max(abs(v) for v in geometry['jaw_surface_gaps_m']))

    def verify_render_pose(self):
        if not self.attached:return
        matrix=UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath('/World/Orange'))
        visual=np.asarray(matrix,dtype=float)[3,:3];physical=self.orange_position()
        p,R=self.ik.grip();expected=p+R@self.grasp_offset
        error=float(np.linalg.norm(visual-physical));grip_error=float(np.linalg.norm(visual-expected))
        self.max_render_pose_error=max(self.max_render_pose_error,error)
        self.max_post_render_grip_error=max(self.max_post_render_grip_error,grip_error);self.render_pose_samples+=1
        if error>.002 or grip_error>.002:
            raise RuntimeError(f'Orange render/physics attachment diverged: render_error={error:.6f} m, grip_error={grip_error:.6f} m')

    def diagnostics(self):
        return dict(render_pose_samples=self.render_pose_samples,max_render_pose_error_m=self.max_render_pose_error,max_post_render_grip_error_m=self.max_post_render_grip_error,engine='PhysX',grasp_assist='kinematic rigid attachment after two-finger geometry check',grasp_samples=self.grasp_samples,max_jaw_surface_gap_m=self.max_grasp_gap,
                    release_height=self.release_height,lowest_after_release=self.lowest_after_release,kinematic_enabled=bool(self.api.GetKinematicEnabledAttr().Get()),
                    orange_position=self.orange_position().tolist(),orange_velocity=self.orange_velocity().tolist())
