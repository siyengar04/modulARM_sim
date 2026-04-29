"""Visualize kinematics and motion trajectory"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pinocchio as pin
from os.path import dirname, join, abspath

# Load saved data
path_data = np.loadtxt("path.txt")
kinematics_data = np.load("kinematics_data.npy", allow_pickle=True)

# Extract end-effector positions
ee_positions = np.array([kd["ee_pos"] for kd in kinematics_data])

print(f"Loaded {len(path_data)} waypoints")
print(f"End-effector trajectory shape: {ee_positions.shape}")

# Create figure with subplots
fig = plt.figure(figsize=(15, 10))

# Plot 1: End-effector trajectory in 3D
ax1 = fig.add_subplot(2, 3, 1, projection="3d")
ax1.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2], "b.-", linewidth=2)
ax1.scatter(
    ee_positions[0, 0],
    ee_positions[0, 1],
    ee_positions[0, 2],
    c="g",
    s=100,
    label="Start",
)
ax1.scatter(
    ee_positions[-1, 0],
    ee_positions[-1, 1],
    ee_positions[-1, 2],
    c="r",
    s=100,
    label="Goal",
)
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")
ax1.set_title("End-Effector Trajectory (3D)")
ax1.legend()
ax1.grid(True)

# Plot 2: XY plane projection
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(ee_positions[:, 0], ee_positions[:, 1], "b.-", linewidth=2)
ax2.scatter(ee_positions[0, 0], ee_positions[0, 1], c="g", s=100, label="Start")
ax2.scatter(ee_positions[-1, 0], ee_positions[-1, 1], c="r", s=100, label="Goal")
ax2.set_xlabel("X (mm)")
ax2.set_ylabel("Y (mm)")
ax2.set_title("End-Effector Trajectory (XY Plane)")
ax2.grid(True)
ax2.legend()
ax2.axis("equal")

# Plot 3: Joint configurations
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(path_data[:, 0], label="Joint 1")
ax3.plot(path_data[:, 1], label="Joint 2")
ax3.set_xlabel("Waypoint")
ax3.set_ylabel("Joint Angle (rad)")
ax3.set_title("Joint Configurations Along Path")
ax3.grid(True)
ax3.legend()

# Plot 4: End-effector distance over time
distances = np.linalg.norm(np.diff(ee_positions, axis=0), axis=1)
ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(distances, "g-", linewidth=1.5)
ax4.set_xlabel("Waypoint")
ax4.set_ylabel("Distance to Next Point (mm)")
ax4.set_title("Step Size Along Path")
ax4.grid(True)

# Plot 5: Cumulative distance
cumulative_distance = np.cumsum(np.concatenate([[0], distances]))
ax5 = fig.add_subplot(2, 3, 5)
ax5.plot(cumulative_distance, "r-", linewidth=2)
ax5.set_xlabel("Waypoint")
ax5.set_ylabel("Cumulative Distance (mm)")
ax5.set_title("Cumulative Path Length")
ax5.grid(True)

# Plot 6: Velocity profile
dt = 0.01  # 10ms per waypoint
velocities = np.linalg.norm(np.diff(path_data, axis=0) / dt, axis=1)
ax6 = fig.add_subplot(2, 3, 6)
ax6.plot(velocities, "purple", linewidth=1.5)
ax6.set_xlabel("Waypoint")
ax6.set_ylabel("Joint Velocity (rad/s)")
ax6.set_title("Joint Velocity Profile")
ax6.grid(True)

plt.tight_layout()
plt.savefig("kinematics_visualization.png")
print("\saved graphs to kinematics_visualization.png")

plt.show()

# Print summary
print("\n" + "=" * 60)
print("VISUALIZATION SUMMARY")
print("=" * 60)
print(f"Total path length: {cumulative_distance[-1]:.2f} mm")
print(f"Average step size: {np.mean(distances):.2f} mm")
print(f"Max step size: {np.max(distances):.2f} mm")
print(f"Max joint velocity: {np.max(velocities):.2f} rad/s")
print(f"Average joint velocity: {np.mean(velocities):.2f} rad/s")
print(f"Total execution time: {len(path_data) * dt:.2f} seconds")
