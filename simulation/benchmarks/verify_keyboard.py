"""Deliver X11 press/release events to our own GLFW window; no global keystrokes."""
from pathlib import Path
import sys,ctypes,time,json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'mujoco'))
from run import Simulation
from window import Window
from common.control import Control
import glfw
import numpy as np
class Key(ctypes.Structure):
    _fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send_event',ctypes.c_int),('display',ctypes.c_void_p),('window',ctypes.c_ulong),('root',ctypes.c_ulong),('subwindow',ctypes.c_ulong),('time',ctypes.c_ulong),('x',ctypes.c_int),('y',ctypes.c_int),('x_root',ctypes.c_int),('y_root',ctypes.c_int),('state',ctypes.c_uint),('keycode',ctypes.c_uint),('same_screen',ctypes.c_int)]
class Event(ctypes.Union):_fields_=[('key',Key),('pad',ctypes.c_long*24)]
x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.restype=ctypes.c_void_p;x.XOpenDisplay.argtypes=[ctypes.c_char_p]
x.XStringToKeysym.restype=ctypes.c_ulong;x.XStringToKeysym.argtypes=[ctypes.c_char_p]
x.XKeysymToKeycode.restype=ctypes.c_uint;x.XKeysymToKeycode.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
x.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.POINTER(Event)];x.XFlush.argtypes=[ctypes.c_void_p];x.XCloseDisplay.argtypes=[ctypes.c_void_p]
display=x.XOpenDisplay(None)
if not display:raise RuntimeError('X11 display unavailable')
s=Simulation();c=Control();w=Window(s.model,c);target=glfw.get_x11_window(w.window)
def send(key,pressed):
    event=Event();event.key.type=2 if pressed else 3;event.key.display=display;event.key.window=target;event.key.same_screen=1
    event.key.keycode=x.XKeysymToKeycode(display,x.XStringToKeysym(key.encode()))
    x.XSendEvent(display,target,False,1 if pressed else 2,ctypes.byref(event));x.XFlush(display)
start=s.data.qpos[:3].copy();stopped=None
try:
    for i in range(130):
        if i==5:send('w',True)
        if i==25:send('a',True)
        if i==65:send('a',False)
        if i==85:send('w',False)
        w.poll();c.tick(s.dt);s.step(c)
        if i==100:stopped=s.data.qpos[:3].copy()
        if i%2==0:w.render(s.data,'X11 press/release regression test')
        time.sleep(.01)
    delta=s.data.qpos[:3]-start
    assert np.linalg.norm(delta[:2])>.08,(delta,c.held)
    assert s.navigation.yaw>np.pi/2+.1,s.navigation.yaw
    assert not c.held,c.held
    assert np.linalg.norm(s.data.qpos[:3]-stopped)<1e-12
    result=dict(passed=True,translation=delta.tolist(),yaw=s.navigation.yaw,held_keys=list(c.held),stop_displacement=float(np.linalg.norm(s.data.qpos[:3]-stopped)))
    print(json.dumps(result));Path('simulation/outputs/keyboard_check.json').write_text(json.dumps(result,indent=2))
finally:w.close();x.XCloseDisplay(display)
