from launch_viewer import Viewer
import numpy as np
import time
import pinocchio as pin
from os.path import dirname, join, abspath
import subprocess
import launch_viewer

urdf_path = "./robot/robot_description.urdf"
mesh_dir = "./robot/meshes"
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_dir)

subprocess.Popen(["pkill", "-f", "gepetto-gui"])
time.sleep(1)
# subprocess.Popen(["gepetto-gui"])
# Viewer.launchViewer(model, collision_model, visual_model)

launch_viewer.Viewer.launchViewer()
# launch_viewer.Viewer.viz.initViewer()
path_data = np.loadtxt("path.txt")
viz = launch_viewer.viz
for config in path_data:
    if viz:
        viz.display(config)
    time.sleep(0.5)
