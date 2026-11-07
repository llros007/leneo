"""Optional wheel-force dynamics; kept separate from LightNav navigation mode."""
import numpy as np
from pxr import Usd,UsdPhysics,PhysxSchema
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from common.control import HOME
from common.navigation import SPAWN,SPAWN_YAW
import math


class Physics:
    dt=.002
    def __init__(self,root,app,timeline):
        for p in Usd.PrimRange(root):
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):PhysxSchema.PhysxArticulationAPI.Apply(p)
            if p.HasAPI(UsdPhysics.CollisionAPI):
                api=PhysxSchema.PhysxCollisionAPI.Apply(p);api.CreateContactOffsetAttr(.001);api.CreateRestOffsetAttr(0)
            if p.IsA(UsdPhysics.Joint):
                name=p.GetName();wheel=name in ['left_wheel_joint','right_wheel_joint'];caster='caster' in name
                drive=UsdPhysics.DriveAPI.Apply(p,'linear' if p.IsA(UsdPhysics.PrismaticJoint) else 'angular')
                drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0 if wheel or caster else 6500 if name=='lift_joint' else 35)
                drive.CreateDampingAttr(1.2 if wheel else .02 if caster else 240 if name=='lift_joint' else 1)
                drive.CreateMaxForceAttr(80 if name=='lift_joint' else 0 if caster else 1.5)
        SimulationManager.set_physics_sim_device('cpu');SimulationManager.set_physics_dt(self.dt)
        self.robot=Articulation(str(root.GetPath()));self.robot.set_solver_iteration_counts(position_counts=16,velocity_counts=4)
        self.names=self.robot.dof_names;self.initial=np.array([[HOME.get(n,0) for n in self.names]],dtype=np.float32)
        self.robot.set_default_state(positions=[list(SPAWN)],orientations=[[math.cos(SPAWN_YAW/2),0,0,math.sin(SPAWN_YAW/2)]],linear_velocities=[[0,0,0]],angular_velocities=[[0,0,0]],dof_positions=self.initial,dof_velocities=0)
        self.positions=self.initial.copy();self.wheels=[self.names.index(n) for n in ['left_wheel_joint','right_wheel_joint']]
        self.ids=[i for i,n in enumerate(self.names) if n in HOME and 'caster' not in n and i not in self.wheels]
        self.speeds=np.array([[.05 if self.names[i]=='lift_joint' else 2 for i in self.ids]])
        self.rates=np.zeros((1,2),dtype=np.float32)
        timeline.play();app.update();timeline.set_auto_update(False);self.robot.reset_to_default_state()
        self.robot.set_dof_position_targets(self.initial)
        self.robot.set_dof_gains(stiffnesses=0,dampings=1.2,dof_indices=self.wheels);self.robot.set_dof_max_velocities(6,dof_indices=self.wheels)

    def step(self,control):
        desired=np.array([[control.targets[self.names[i]] for i in self.ids]])
        self.positions[:,self.ids]+=np.clip(desired-self.positions[:,self.ids],-self.speeds*self.dt,self.speeds*self.dt)
        self.rates+=np.clip(np.asarray([control.wheels()])-self.rates,-12*self.dt,12*self.dt)
        self.robot.set_dof_position_targets(self.positions[:,self.ids],dof_indices=self.ids)
        self.robot.set_dof_velocity_targets(self.rates,dof_indices=self.wheels)
        SimulationManager.step()

    def state(self):
        position,rotation=self.robot.get_world_poses()
        values=self.robot.get_dof_positions().numpy()[0]
        return position.numpy()[0],rotation.numpy()[0],{n:float(values[i]) for i,n in enumerate(self.names)}
