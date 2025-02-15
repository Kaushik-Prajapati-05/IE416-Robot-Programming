#-------- Example-4---------#
# Collotion geometry

import roboticstoolbox as rtb
import swift
import numpy as np
import spatialmath as sm
import spatialgeometry as sg

# Create swift instance
env =swift.Swift()
env.launch(realtime=True)

# Define the robot model
robot = rtb.models.Panda()
robot.q = robot.qr

# Add robot to swift
env.add(robot, robot_alpha=0.5, collision_alpha=0.5)

# Set goal position
goal = robot.fkine(robot.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.35)
axes = sg.Axes(length=0.1, base=goal)
env.add(axes)

# Arrived at a destination flag
arrived = False

# Time step
dt = 0.01

while not arrived:
    # v is a 6 vector representing the spatial error
    v, arrived = rtb.p_servo(robot.fkine(robot.q), goal, gain=0.1, threshold=0.01)
    J = robot.jacobe(robot.q)
    robot.qd = np.linalg.pinv(J) @ v

    # Step the environment
    env.step(dt)

# Stop the browser tab from closing
env.hold()