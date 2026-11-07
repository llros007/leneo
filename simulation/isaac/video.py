"""Capture completed RTX viewport frames without changing simulation time."""
import ctypes
import numpy as np
from common.video import VideoWriter


class ViewportVideo:
    def __init__(self,path,width,height,fps,app,orbit,orange):
        self.writer=VideoWriter(path,width,height,fps,'Isaac Sim 6.2')
        self.app=app;self.orbit=orbit;self.orange=orange
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_buffer
        self.viewport=get_active_viewport();self.capture_to_buffer=capture_viewport_to_buffer
        self.pointer=ctypes.pythonapi.PyCapsule_GetPointer
        self.pointer.argtypes=[ctypes.py_object,ctypes.c_char_p];self.pointer.restype=ctypes.c_void_p
        for _ in range(15):self.update()

    @property
    def next_time(self):return self.writer.next_time

    def update(self):
        self.app.update()
        if self.orange:self.orange.verify_render_pose()

    def capture(self,simtime,phase):
        self.orbit.update(self.orbit.position)
        received=[]
        def callback(buffer,size,width,height,byte_format):
            if size!=width*height*4:raise RuntimeError(f'Unexpected capture format {byte_format}, {size} bytes')
            pointer=self.pointer(buffer,None)
            raw=ctypes.string_at(pointer,size)
            received.append(np.frombuffer(raw,dtype=np.uint8).reshape(height,width,4))
        ticket=self.capture_to_buffer(self.viewport,callback)
        for _ in range(120):
            self.update()
            if received:break
        if not received:raise RuntimeError('RTX video capture timed out')
        self.writer.write(received[0],simtime,phase)
        if self.writer.frames%300==0:print(f'Video: {simtime:.1f} seconds, {self.writer.frames} frames',flush=True)

    def close(self):self.writer.close()

    def report(self):return self.writer.report()
