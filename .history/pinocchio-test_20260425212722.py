#!/usr/bin/env python3
import pinocchio as pin
import numpy as np
import rospy


class RobotSimulator:
    def __init__(self, urdf_path):
        # Load model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Joint limits and velocity limits
        self.q = pin.neutral(self.model)  # Configuration
        self.v = np.zeros(self.model.nv)  # Velocity
        self.tau = np.zeros(self.model.nv)  # Torques

        # Gravity
        self.gravity = pin.Physics.StandardGravity()

    def step(self, motor_commands, dt):
        """Forward dynamics simulation step"""
        # Apply motor torques (simplified)
        self.tau[:] = motor_commands

        # Compute forward dynamics
        pin.forwardDynamics(self.model, self.data, self.q, self.v, self.tau)

        # Semi-implicit Euler integration
        self.v += self.data.ddq * dt
        self.q = pin.integrate(self.model, self.q, self.v * dt)

        return self.q, self.v

    def get_end_effector_position(self, frame_name):
        """Compute forward kinematics for any frame"""
        frame_id = self.model.getFrameId(frame_name)
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[frame_id].translation
