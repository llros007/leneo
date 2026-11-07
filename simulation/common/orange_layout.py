"""Shared metre/Z-up layout for both floor-to-bar demonstrations."""
import numpy as np
SPAWN=(-1.25,.1295,.008)
ORANGE=np.array([-.95,.30,.02319412875])
TABLE_TOP=.9080405552
PLACE=np.array([-.95,.96,TABLE_TOP+.034])
GRIP=np.array([-.0276954,0,.003])
NAMES=['lift_joint']+['right_'+s for s in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']]
