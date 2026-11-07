"""Viewport-only mouse orbit/pan/zoom. Manual movement suspends following."""
import math
import copy
import numpy as np
from pxr import Gf


class OrbitCamera:
    def __init__(self,op,position,attach=False):
        self.status=None;self.op=op;self.buttons=set();self.mouse=None;self.frame=None
        self.on_manual=None
        self.native_bindings=None;self.rejected_jumps=0
        self.position=np.asarray(position,dtype=float).copy();self.previous=None
        self.reset(position)
        if attach:self.attach()

    def reset(self,position=None):
        if self.on_manual:self.on_manual()
        if position is not None:self.position=np.asarray(position,dtype=float).copy()
        self.target=self.position+np.array([0,0,.5]);self.distance=1.65
        self.yaw=math.atan2(.8,.9);self.pitch=math.radians(28)
        self.follow=True;self.dirty=True;self.update(self.position)

    def toggle_follow(self):
        if self.on_manual:self.on_manual()
        self.follow=not self.follow
        self.previous=self.position.copy();self.dirty=True

    def press(self,x,y,button,modifiers=0):
        if self.on_manual:self.on_manual()
        self.buttons.add(button);self.mouse=(x,y);self.follow=False

    def release(self,x,y,button,modifiers=0):
        self.buttons.discard(button);self.mouse=(x,y)

    def move(self,x,y,*_):
        if self.mouse is None:self.mouse=(x,y)
        dx,dy=x-self.mouse[0],y-self.mouse[1];self.mouse=(x,y)
        if not self.buttons:return
        # Right-button flight/cursor capture can warp the absolute position.
        # Re-anchor to the new position without turning a warp into a camera jump.
        if max(abs(dx),abs(dy))>120:
            self.rejected_jumps+=1
            return
        magnitude=math.hypot(dx,dy)
        if magnitude>40:dx*=40/magnitude;dy*=40/magnitude
        if 0 in self.buttons:
            self.yaw-=dx*.0035
            self.pitch=max(math.radians(-80),min(math.radians(85),self.pitch+dy*.0035))
        else:
            right=np.array([-math.sin(self.yaw),math.cos(self.yaw),0])
            up=np.array([-math.cos(self.yaw)*math.sin(self.pitch),-math.sin(self.yaw)*math.sin(self.pitch),math.cos(self.pitch)])
            self.target+=(right*(-dx)+up*dy)*self.distance*.0008
        self.dirty=True

    def wheel(self,x,y,*_):
        if self.on_manual:self.on_manual()
        self.follow=False;self.distance=max(.25,min(12,self.distance*math.exp(-max(-4,min(4,y))*.12)));self.dirty=True

    def apply_demo_pose(self,pose):
        self.target=np.array(pose.lookat,copy=True);self.distance=float(pose.distance)
        self.yaw=math.radians(pose.azimuth-180);self.pitch=math.radians(-pose.elevation)
        self.follow=False;self.dirty=True

    def update(self,position):
        position=np.asarray(position,dtype=float)
        if self.previous is not None and self.follow:
            delta=position-self.previous
            if np.any(delta):self.target+=delta;self.dirty=True
        self.previous=position.copy();self.position=position.copy()
        if not self.dirty:return
        offset=self.distance*np.array([math.cos(self.pitch)*math.cos(self.yaw),math.cos(self.pitch)*math.sin(self.yaw),math.sin(self.pitch)])
        eye=self.target+offset
        self.op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,self.target)),Gf.Vec3d(0,0,1)).GetInverse())
        self.dirty=False

    def attach(self):
        import omni.ui as ui
        import carb.settings
        settings=carb.settings.get_settings()
        binding_path='/exts/omni.kit.viewport.window/bindings/camera'
        self.native_bindings=copy.deepcopy(settings.get(binding_path))
        # An opaque UI overlay alone does not disable ui.scene flight gestures.
        # Only this controller should handle mouse gestures in this application.
        settings.set(binding_path,{})
        from omni.kit.viewport.utility import get_active_viewport_window
        window=get_active_viewport_window()
        self.frame=window.get_frame('leneo.mouse_camera')
        with self.frame:
            with ui.ZStack():
                self.surface=ui.Rectangle(style={'background_color':0x00000000},opaque_for_mouse_events=True,
                    mouse_pressed_fn=self.press,mouse_released_fn=self.release,mouse_moved_fn=self.move,
                    mouse_wheel_fn=self.wheel,scroll_only_window_hovered=True)
                with ui.VStack():
                    ui.Label('Mouse: left orbit | right/middle pan | wheel zoom | V follow | B reset',height=22,
                        style={'color':0xfff4f4f4,'font_size':14},opaque_for_mouse_events=False)
                    self.status=ui.Label('',height=22,style={'color':0xfff4f4f4,'font_size':14},opaque_for_mouse_events=False)
                    ui.Spacer()

    def close(self):
        self.buttons.clear()
        if self.native_bindings is not None:
            import carb.settings
            carb.settings.get_settings().set('/exts/omni.kit.viewport.window/bindings/camera',self.native_bindings)
            self.native_bindings=None
        if self.frame:self.frame.clear();self.frame=None
