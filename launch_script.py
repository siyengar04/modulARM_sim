from ompltest import modulARMPlanner
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import sys
import os
from os.path import dirname, join, abspath
from archive.launch_viewer import Viewer

urdf_model_path = join(dirname(abspath(__file__)), "robot/robot_description.urdf")
mesh_dir = join(dirname(abspath(__file__)), "robot/meshes")
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir
)
viz = Viewer.launchViewer(model, collision_model, visual_model)
start_config = [0.0] * 2
goal_config = [0.5] * 2
# viz.display(pin.neutral(model))

planner = modulARMPlanner()

print(f"DOF: {planner.model.nq}")
print(f"lower limits: {planner.model.lowerPositionLimit}")
print(f"upper limits: {planner.model.upperPositionLimit}")
solved, path = planner.plan(start_config, goal_config)

if solved:
    print("Solution found!")
    print("Path:", path)

    path.interpolate()

    states = path.getStates()
    path_data = np.array(
        [
            [float(states[i][j]) for j in range(planner.model.nq)]
            for i in range(len(states))
        ]
    )

    print(f"Path with {len(states)} states:")
    print(path_data)

    np.savetxt("path.txt", path_data)

    if viz:
        for state in path_data:
            print(f"Displaying state: {state}")
            import time

            time.sleep(0.1)

    if planner.model.nq == 2:
        plt.figure()
        plt.plot(path_data[:, 0], path_data[:, 1], "r.-")
        plt.xlabel("Joint 1")
        plt.ylabel("Joint 2")
        plt.title("Motion Plan")
        plt.savefig("path.png")
        plt.show()
else:
    print("No solution found")
