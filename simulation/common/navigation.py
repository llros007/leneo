"""LightNav-style planar navigation with swept footprint checks against the kitchen.

This mode prescribes the base and joint poses; it does not simulate wheel forces.
The separate physics mode retains the original floating-base dynamics.
"""
import json
import math
import numpy as np
from .control import ROOT, HOME, PARAMS

SPAWN = (-1.25,-1.35,0.008)
SPAWN_YAW = math.pi/2


class Navigation:
    def __init__(self, position=SPAWN, yaw=SPAWN_YAW):
        self.position=np.array(position,dtype=float)
        self.yaw=float(yaw)
        self.joints=dict(HOME)
        self.blocked=False
        self.blocked_steps=0
        boxes=json.loads((ROOT/'assets/modern_kitchen/manifest.json').read_text())['collision_boxes']
        boxes=[b for b in boxes if b['center'][2]+b['half'][2]>.045 and b['center'][2]-b['half'][2]<.25]
        self.centers=np.array([b['center'][:2] for b in boxes])
        self.half=np.array([b['half'][:2] for b in boxes])
        if self.collides(self.position[:2],self.yaw):raise ValueError('Navigation spawn overlaps kitchen collision proxy')

    def collides(self,position,yaw):
        # SAT: chassis rectangle against static axis-aligned kitchen boxes.
        c,s=math.cos(yaw),math.sin(yaw);a,b=.31,.265
        delta=self.centers-np.asarray(position)
        overlap=(np.abs(delta[:,0]) < self.half[:,0]+a*abs(c)+b*abs(s))
        overlap&=(np.abs(delta[:,1]) < self.half[:,1]+a*abs(s)+b*abs(c))
        overlap&=(np.abs(delta[:,0]*c+delta[:,1]*s) < a+self.half[:,0]*abs(c)+self.half[:,1]*abs(s))
        overlap&=(np.abs(-delta[:,0]*s+delta[:,1]*c) < b+self.half[:,0]*abs(s)+self.half[:,1]*abs(c))
        return bool(np.any(overlap))

    def step(self,control,dt):
        left,right=control.wheels()
        v=(left+right)*PARAMS['wheel_radius_m']/2
        w=(right-left)*PARAMS['wheel_radius_m']/PARAMS['wheel_track_m']
        heading=self.yaw+w*dt/2
        candidate=self.position[:2]+v*dt*np.array([math.cos(heading),math.sin(heading)])
        yaw=math.atan2(math.sin(self.yaw+w*dt),math.cos(self.yaw+w*dt))
        self.blocked=self.collides(candidate,yaw)
        if self.blocked:self.blocked_steps+=1
        else:
            self.position[:2]=candidate;self.yaw=yaw
            self.joints['left_wheel_joint']+=left*dt;self.joints['right_wheel_joint']+=right*dt
        for name,target in control.targets.items():
            if name.endswith('wheel_joint') or 'caster' in name:continue
            speed=.05 if name=='lift_joint' else 2.0
            self.joints[name]+=float(np.clip(target-self.joints[name],-speed*dt,speed*dt))

    @property
    def quaternion(self):return [math.cos(self.yaw/2),0,0,math.sin(self.yaw/2)]
