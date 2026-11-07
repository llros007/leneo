"""Phase-triggered camera cuts for the floor-to-bar demonstration."""
import numpy as np
from types import SimpleNamespace

PICK_PHASES={'floor-grasp','close','grasp','lift-orange'}
PLACE_PHASES={'above-bar','place','release','open','retract','settle','complete'}


def camera_default():
    camera=SimpleNamespace(lookat=np.zeros(3),distance=0.,azimuth=0.,elevation=0.)
    camera.lookat[:]=[-.70,1.10,.60]
    camera.distance=2.25;camera.azimuth=150;camera.elevation=-28
    return camera


def camera_preset(name):
    camera=camera_default()
    if name=='pick':
        camera.lookat[:]=[-.98,.55,.22]
        camera.distance=1.30;camera.azimuth=150;camera.elevation=-28
    elif name=='place':
        camera.lookat[:]=[-1.20,.90,.82]
        camera.distance=1.45;camera.azimuth=200;camera.elevation=-42
    elif name!='default':raise ValueError(name)
    return camera


def recording_default():
    camera=camera_default();camera.lookat[:]=[-1.10,.65,.55];camera.distance=2.65
    return camera


class DemoCamera:
    def __init__(self,camera,duration=.65,overrides=None):
        self.camera=camera;self.duration=duration;self.cue='default';self.transition=None;self.events=[]
        self.overrides=overrides or {}
        if 'default' in self.overrides:
            pose=self.overrides['default'];camera.lookat[:]=pose.lookat
            camera.distance=pose.distance;camera.azimuth=pose.azimuth;camera.elevation=pose.elevation

    @staticmethod
    def cue_for_phase(phase):
        if phase in PICK_PHASES:return 'pick'
        if phase in PLACE_PHASES:return 'place'
        return 'default'

    @staticmethod
    def values(camera):
        return np.r_[camera.lookat,camera.distance,camera.azimuth,camera.elevation]

    def manual(self):
        # Leave mouse edits alone until the next task camera cue.
        self.transition=None

    def update(self,phase,now):
        cue=self.cue_for_phase(phase);changed=cue!=self.cue
        if changed:
            self.cue=cue;start=self.values(self.camera);target=self.values(self.overrides.get(cue,camera_preset(cue)))
            target[4]=start[4]+(target[4]-start[4]+180)%360-180
            self.transition=(float(now),start,target)
            self.events.append(dict(time=float(now),phase=phase,view=cue))
        if self.transition:
            since,start,target=self.transition
            t=float(np.clip((now-since)/self.duration,0,1));u=t*t*(3-2*t)
            value=start+(target-start)*u
            self.camera.lookat[:]=value[:3];self.camera.distance=value[3];self.camera.azimuth=value[4];self.camera.elevation=value[5]
            if t>=1:self.transition=None
        return changed
