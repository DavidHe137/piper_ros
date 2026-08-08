from piper_sdk import *
import numpy as np
np.set_printoptions(formatter={'all':lambda x: str(x)})

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()

    # Two opposite corners of an axis-aligned box (order per axis does not matter).
    safety_box_corner_a = np.array([0.057, 0.3, 0.092])
    safety_box_corner_b = np.array([0.611, -0.275, 0.607])
    box_min = np.minimum(safety_box_corner_a, safety_box_corner_b)
    box_max = np.maximum(safety_box_corner_a, safety_box_corner_b)

    while True:
        endpose = piper.GetArmEndPoseMsgs().end_pose
        x_axis_trunc = round(endpose.X_axis * 1e-6, 3) 
        y_axis_trunc = round(endpose.Y_axis * 1e-6, 3)
        z_axis_trunc = round(endpose.Z_axis * 1e-6, 3)
        endpose_arr = np.array([x_axis_trunc, y_axis_trunc, z_axis_trunc])

        # Inside iff every axis lies in [min, max] for that axis (Y had min -0.249, max 0.3).
        is_outside_safety_box = np.any(endpose_arr < box_min) or np.any(endpose_arr > box_max)

        print(endpose_arr, is_outside_safety_box)