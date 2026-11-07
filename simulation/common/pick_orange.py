"""Shared floor-to-bar task with engine-native IK and geometry-checked grasp assistance."""
import math
import time
import numpy as np
from common.control import HOME
from common.orange_layout import ORANGE, PLACE, TABLE_TOP, NAMES



class PickOrange:
    def __init__(self,sim,control):
        self.sim=sim;self.control=control;self.phase='start';self.since=0.;self.index=-1
        self.events=[];self.ik_errors=[];self.done=False;self.success=False;self.released=False
        self.actions=[('arm','pregrasp',ORANGE+[0,0,.10]),('arm','floor-grasp',ORANGE),('grip','close',.51683),('attach','grasp',None),
            ('arm','lift-orange',ORANGE+[0,0,.18]),('joints','carry',dict(HOME,lift_joint=.05,right_gripper=.51683)),
            ('drive','approach-bar',(-1.25,.50,math.pi/2,False)),
            ('joints','raise-to-bar',dict(HOME,lift_joint=.36404873,right_gripper=.51683)),('arm','above-bar',PLACE+[0,0,.10]),('arm','place',PLACE),
            ('release','release',None),('grip','open',1.45),('arm','retract',PLACE+[0,0,.14]),('wait','settle',3.)]
        self.advance()

    def ik(self,target):
        result,error=self.sim.solve_ik(target)
        self.ik_errors.append(float(error))
        if error>.008:raise RuntimeError(f'IK failed for {self.phase}: {error:.4f} m at {target}')
        return result

    def advance(self):
        self.index+=1;self.since=float(self.sim.time)
        if self.index==len(self.actions):
            self.done=True;self.phase='complete';self.check();return
        kind,self.phase,value=self.actions[self.index]
        self.events.append(dict(phase=self.phase,time=self.since,orange=self.sim.orange_position().tolist()))
        print(f'[pick-orange {self.since:.2f}s] {self.phase}',flush=True)
        self.start=dict(self.control.targets);self.goal=dict(self.start);self.duration=.5
        if kind=='arm':
            target=self.sim.orange_position().copy() if self.phase=='floor-grasp' else np.asarray(value)
            self.goal.update(self.ik(target))
        elif kind=='joints':self.goal.update(value)
        elif kind=='grip':self.goal['right_gripper']=value;self.duration=1.
        if kind in ['arm','joints']:
            self.duration=max(1.,max(abs(self.goal[n]-self.start[n])/(.04 if n=='lift_joint' else .65) for n in self.goal)*1.6)
        if kind=='attach':self.sim.grasp(True)
        if kind=='release':
            self.control.targets['right_gripper']=1.45
            self.sim.grasp(False);self.released=True

    def tick(self):
        c=self.control;s=self.sim;c.linear=c.angular=0;c.last_drive=time.monotonic()
        if self.done:return
        elapsed=s.time-self.since;kind,_,value=self.actions[self.index]
        if elapsed>45:raise RuntimeError(f'Demo timed out at {self.phase}; blocked={s.navigation.blocked}')
        if kind=='drive':
            x,y,yaw,reverse=value;delta=np.array([x,y])-s.navigation.position[:2];dist=np.linalg.norm(delta)
            wrap=lambda a:math.atan2(math.sin(a),math.cos(a))
            heading=math.atan2(delta[1],delta[0])+(math.pi if reverse else 0) if dist>.012 else yaw
            angle=wrap(heading-s.navigation.yaw)
            c.angular=float(np.clip(angle*2,-.5,.5))
            if dist>.012 and abs(angle)<.12:c.linear=(-1 if reverse else 1)*min(.13,dist*.8)
            if dist<.012 and abs(wrap(yaw-s.navigation.yaw))<.02:self.advance()
        elif kind in ['arm','joints','grip']:
            fraction=min(1.,elapsed/self.duration);u=fraction*fraction*(3-2*fraction)
            for n in self.goal:c.targets[n]=self.start[n]+u*(self.goal[n]-self.start[n])
            if elapsed>=self.duration+.2:self.advance()
        elif kind=='wait':
            if elapsed>=value:self.advance()
        elif elapsed>=.5:self.advance()

    def check(self):
        s=self.sim;q=s.orange_position();v=s.orange_velocity()
        self.success=bool(self.released and not s.attached and np.linalg.norm(q[:2]-PLACE[:2])<.06 and abs(q[2]-(TABLE_TOP+.02219412875))<.012 and np.linalg.norm(v)<.03)
        if not self.success:raise RuntimeError(f'Orange did not settle on bar: position={q}, velocity={v}')

    def report(self):
        s=self.sim
        return dict(name='pick-orange',phase=self.phase,complete=self.done,success=self.success,assisted_grasp=True,attached=s.attached,released=self.released,grasp_geometry=s.grasp_geometry,events=self.events,ik_position_errors_m=self.ik_errors,orange_position=s.orange_position().tolist(),orange_velocity=s.orange_velocity().tolist(),table_height=TABLE_TOP)
