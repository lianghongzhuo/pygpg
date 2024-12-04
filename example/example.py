import numpy as np
import pygpg
points = np.loadtxt("example/box.txt")
num_samples = 100
show_grasp = True
gripper_config_file = "gripper_params.cfg"
grasps = pygpg.generate_grasps(points, num_samples, show_grasp, gripper_config_file)
