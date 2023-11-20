import numpy as np
import pygpg
import open3d as o3d

# create a open3d box mesh
box = o3d.geometry.TriangleMesh.create_box(width=0.04, height=0.22, depth=0.06)
# sample points from the mesh
points = np.asarray(box.sample_points_uniformly(number_of_points=10000).points)
num_samples = 100
show_grasp = True
gripper_config_file = "gripper_params.cfg"
grasps = pygpg.generate_grasps(points, num_samples, show_grasp, gripper_config_file)
