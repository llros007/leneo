"""Minimal GLFW window: direct press/release input, no stock viewer menus/shortcuts."""
import time
import numpy as np
import glfw
import mujoco


from demo_camera import camera_default


class Window:
    def __init__(self,model,control,width=960,height=640,quality='fast'):
        if not glfw.init():raise RuntimeError('GLFW could not initialize; check DISPLAY')
        glfw.window_hint(glfw.SAMPLES,4)
        self.window=glfw.create_window(width,height,'LeNeo | WASD/arrows drive | Q/E lift | Esc quit',None,None)
        if not self.window:glfw.terminate();raise RuntimeError('Cannot create OpenGL window')
        glfw.make_context_current(self.window);glfw.swap_interval(0)
        self.model=model;self.control=control;self.quality=quality
        self.context=mujoco.MjrContext(model,mujoco.mjtFontScale.mjFONTSCALE_100)
        self.scene=mujoco.MjvScene(model,maxgeom=5000);self.camera=camera_default()
        self.option=mujoco.MjvOption();self.option.geomgroup[0]=self.option.geomgroup[3]=0
        self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=quality=='full'
        self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
        self.demo_camera=None
        self.follow=False;self.mouse=glfw.get_cursor_pos(self.window)
        aliases={glfw.KEY_UP:'W',glfw.KEY_DOWN:'S',glfw.KEY_LEFT:'A',glfw.KEY_RIGHT:'D',glfw.KEY_TAB:'TAB',glfw.KEY_ESCAPE:'ESCAPE',glfw.KEY_SPACE:'SPACE'}
        def key(win,key,scan,action,mods):
            name=aliases.get(key,chr(key) if 32<=key<127 else '')
            if action==glfw.RELEASE:control.release(name)
            elif action==glfw.PRESS:
                if key==glfw.KEY_C:self.option.geomgroup[3]=1-self.option.geomgroup[3]
                elif key==glfw.KEY_V:
                    if self.demo_camera:self.demo_camera.manual()
                    self.follow=not self.follow
                else:control.press(name)
        def move(win,x,y):
            old=self.mouse;self.mouse=(x,y)
            left=glfw.get_mouse_button(win,glfw.MOUSE_BUTTON_LEFT)==glfw.PRESS
            right=glfw.get_mouse_button(win,glfw.MOUSE_BUTTON_RIGHT)==glfw.PRESS
            if not(left or right):return
            if self.demo_camera:self.demo_camera.manual()
            action=mujoco.mjtMouse.mjMOUSE_ROTATE_V if left else mujoco.mjtMouse.mjMOUSE_MOVE_V
            if right:self.follow=False
            height=max(1,glfw.get_window_size(win)[1]);mujoco.mjv_moveCamera(model,action,(x-old[0])/height,(y-old[1])/height,self.camera)
        glfw.set_key_callback(self.window,key)
        glfw.set_window_focus_callback(self.window,lambda win,focused:None if focused else control.clear_keys())
        glfw.set_cursor_pos_callback(self.window,move)
        def scroll(win,x,y):
            if self.demo_camera:self.demo_camera.manual()
            mujoco.mjv_moveCamera(model,mujoco.mjtMouse.mjMOUSE_ZOOM,0,-.05*y,self.camera)
        glfw.set_scroll_callback(self.window,scroll)

    def poll(self):
        glfw.poll_events()
        return not glfw.window_should_close(self.window)

    def render(self,data,status):
        glfw.make_context_current(self.window)
        if self.follow:self.camera.lookat[:]=data.qpos[:3]+[0,0,.5]
        width,height=glfw.get_framebuffer_size(self.window)
        if width<=0 or height<=0:return
        viewport=mujoco.MjrRect(0,0,width,height)
        mujoco.mjv_updateScene(self.model,data,self.option,None,self.camera,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
        self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=self.quality=='full'
        self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
        mujoco.mjr_render(viewport,self.scene,self.context)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,viewport,
            'WASD/arrows: drive+turn | release: brake | Q/E: lift\nTab: arm | 1..6: joint | J/K: move | H: home | V: follow | Esc: quit',status,self.context)
        glfw.swap_buffers(self.window)

    def close(self):
        self.control.clear_keys();self.context.free();glfw.destroy_window(self.window);glfw.terminate()
