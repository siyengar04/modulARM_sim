from ompl import base as ob
from ompl import geometric as og
import pinocchio as pin
import numpy as np


class RobotMotionPlanner:
    def __init__(self, urdf_path):
        # Load robot model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Create state space matching robot DOF
        self.space = ob.RealVectorStateSpace(self.model.nq)

        # Set joint limits
        bounds = ob.RealVectorBounds(self.model.nq)
        for i in range(self.model.nq):
            bounds.setLow(i, self.model.lowerPositionLimit[i])
            bounds.setHigh(i, self.model.upperPositionLimit[i])
        self.space.setBounds(bounds)

        # Create space information
        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(self.isStateValid)

    def isStateValid(self, state):
        # Convert OMPL state to configuration
        q = np.array([state[i] for i in range(self.model.nq)])

        # Check for collisions or other constraints
        pin.forwardKinematics(self.model, self.data, q)

        # Add your collision checking here
        return True  # Valid if no collisions

    def plan(self, start_config, goal_config, time_limit=1.0):
        pdef = ob.ProblemDefinition(self.si)

        # Set start and goal
        start = ob.State(self.space)
        goal = ob.State(self.space)
        for i in range(self.model.nq):
            start[i] = start_config[i]
            goal[i] = goal_config[i]

        pdef.setStartAndGoalStates(start, goal)

        # Use planner
        planner = og.RRTConnect(self.si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        solved = planner.solve(time_limit)
        return solved, pdef.getSolutionPath() if solved else None


# Usage
planner = RobotMotionPlanner("robot_description.urdf")
solved, path = planner.plan(start_config, goal_config)
