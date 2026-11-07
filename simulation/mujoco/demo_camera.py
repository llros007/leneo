"""MuJoCo adapter for the shared task camera director."""
import mujoco
from common.demo_camera import DemoCamera, camera_preset, PICK_PHASES, PLACE_PHASES

def camera_default():
    pose=camera_preset('default');camera=mujoco.MjvCamera()
    camera.lookat[:]=pose.lookat;camera.distance=pose.distance
    camera.azimuth=pose.azimuth;camera.elevation=pose.elevation
    return camera
