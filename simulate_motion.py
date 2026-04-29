import subprocess
from ompltest import modulARMPlanner
import numpy as np
import pinocchio as pin
from os.path import dirname, join, abspath
import time
from launch_viewer import Viewer
from pinocchio.visualize import GepettoVisualizer

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
    print("\n✓ Solution found!")

    path.interpolate()
    states = path.getStates()
    path_data = np.array(
        [
            [float(states[i][j]) for j in range(planner.model.nq)]
            for i in range(len(states))
        ]
    )

    print(f"Path contains {len(path_data)} waypoints\n")

    print("\nAvailable frames in model:")
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

    print(
        f"Using end-effector frame: {model.frames[ee_frame_id].name} (id={ee_frame_id})\n"
    )

    print(
        f"{'Step':<6} {'Joint 1':<12} {'Joint 2':<12} {'EE X':<12} {'EE Y':<12} {'EE Z':<12}"
    )

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

    print(f"Start configuration: {start_config}")
    print(f"Goal configuration:  {goal_config}")
    print(f"\nStart EE position: {start_ee}")
    print(f"End EE position:   {end_ee}")
    print(f"Total EE displacement: {np.linalg.norm(end_ee - start_ee):.4f}")

    print("JOINT VELOCITIES (finite differences, assuming 0.01s timestep)")
    print(f"{'Step':<6} {'Joint 1 Vel':<15} {'Joint 2 Vel':<15} {'Total Vel':<15}")

    dt = 0.01
    for step in range(1, len(path_data), max(1, len(path_data) // 10)):
        dq = (path_data[step] - path_data[step - 1]) / dt
        total_vel = np.linalg.norm(dq)
        print(f"{step:<6} {dq[0]:<15.4f} {dq[1]:<15.4f} {total_vel:<15.4f}")

    np.save("kinematics_data.npy", kinematics_data, allow_pickle=True)
    np.savetxt("path.txt", path_data)

    print("\n✓ Kinematics data saved to kinematics_data.npy")
    print("✓ Path data saved to path.txt")
    for config in path_data:
        viz.display(config)

        time.sleep(0.500)  # 50ms per frame for smooth playback

# To visualize the motion in Gepetto:
# 1. Start gepetto-gui in another terminal
# 2. Run the following code:

#     from launch_viewer import Viewer
#     import numpy as np
#     path_data = np.loadtxt('path.txt')

#     # Display the motion
#     for config in path_data:
#         viz.display(config)
#         time.sleep(0.05)  # 50ms per frame


else:
    print("✗ No solution found")
