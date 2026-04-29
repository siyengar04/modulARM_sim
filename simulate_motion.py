import subprocess
from ompltest import modulARMPlanner
import numpy as np
import pinocchio as pin
from os.path import dirname, join, abspath
import time
from launch_viewer import Viewer
from pinocchio.visualize import GepettoVisualizer

tDelayS = 0.05

urdf_model_path = join(dirname(abspath(__file__)), "robot/robot_description.urdf")
mesh_dir = join(dirname(abspath(__file__)), "robot/meshes")

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir
)
subprocess.Popen(["gepetto-gui"])
viz = GepettoVisualizer(model, collision_model, visual_model)
viz.initViewer()
viz.loadViewerModel("pinocchio")

data = model.createData()

planner = modulARMPlanner()
start_config = np.array([0.0] * planner.model.nq)
goal_config = np.array([0.5] * planner.model.nq)


solved, path = planner.plan(start_config, goal_config)

if solved:
    print("\nsolution exists!")

    path.interpolate()
    states = path.getStates()
    path_data = np.array(
        [
            [float(states[i][j]) for j in range(planner.model.nq)]
            for i in range(len(states))
        ]
    )

    print(f"{len(path_data)} waypoints generated\n")

    print("\frames in model:")
    for i, frame in enumerate(model.frames):
        print(f"  {i}: {frame.name}")

    ee_frame_id = 19
    try:

        for i, frame in enumerate(model.frames):
            if "EE" in frame.name.upper() or "end" in frame.name.lower():
                ee_frame_id = i
                break
    except:
        pass

    print(f"end-effector frame: {model.frames[ee_frame_id].name} (id={ee_frame_id})\n")

    print(f"{'t-step':<6} {'j1':<12} {'j2':<12} {'EE X':<12} {'EE Y':<12} {'EE Z':<12}")

    kinematics_data = []

    for step, config in enumerate(path_data):
        pin.forwardKinematics(model, data, config)
        pin.updateFramePlacements(model, data)

        ee_placement = data.oMf[ee_frame_id]
        ee_pos = ee_placement.translation

        ee_rot = ee_placement.rotation

        kinematics_data.append(
            {
                "step": step,
                "config": config.copy(),
                "ee_pos": ee_pos.copy(),
                "ee_rot": ee_rot.copy(),
                "ee_placement": ee_placement,
            }
        )

        if step % max(1, len(path_data) // 10) == 0 or step == len(path_data) - 1:
            print(
                f"{step:<6} {config[0]:<12.4f} {config[1]:<12.4f} {ee_pos[0]:<12.4f} {ee_pos[1]:<12.4f} {ee_pos[2]:<12.4f}"
            )

    start_ee = kinematics_data[0]["ee_pos"]
    end_ee = kinematics_data[-1]["ee_pos"]

    print(f"initial: {start_config}")
    print(f"goal:  {goal_config}")
    print(f"\ninitial EE position: {start_ee}")
    print(f"final EE position:   {end_ee}")
    print(f"EE displacement: {np.linalg.norm(end_ee - start_ee):.4f}")

    print("theta_dot @ 0.01s timestep")
    print(f"{'t-step':<6} {'j1 vel':<15} {'j2 vel':<15}")

    dt = 0.01

    for step in range(1, len(path_data), max(1, len(path_data) // 10)):
        dq = (path_data[step] - path_data[step - 1]) / dt
        total_vel = np.linalg.norm(dq)
        print(f"{step:<6} {dq[0]:<15.4f} {dq[1]:<15.4f} {total_vel:<15.4f}")

    np.save("kinematics_data.npy", kinematics_data, allow_pickle=True)
    print("\nsaved to kinematics_data.npy")

    np.savetxt("path.txt", path_data)

    print("saved to path.txt")
    for config in path_data:
        viz.display(config)

        time.sleep(tDelayS)


else:
    print("no solution")
