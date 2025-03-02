import swift
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import pandas as pd

# Initialize simulation
env = swift.Swift()
env.launch()

# Create Panda robot
panda = rtb.models.Panda()
panda.q = panda.qr  # Set initial joint angles

# Define two moving obstacles
s0 = sg.Sphere(radius=0.05, pose=sm.SE3(0.52, 0.4, 0.3))
s0.v = [0, -0.2, 0, 0, 0, 0]

s1 = sg.Sphere(radius=0.05, pose=sm.SE3(0.1, 0.35, 0.65))
s1.v = [0, -0.2, 0, 0, 0, 0]

# Define target position
target = sg.Sphere(radius=0.02, pose=sm.SE3(0.6, -0.2, 0.0))

# Add objects to environment
env.add(panda)
env.add(s0)
env.add(s1)
env.add(target)

# Set target pose
Tep = panda.fkine(panda.q)
Tep.A[:3, 3] = target.T[:3, -1]

# Data storage
data = []

# Function to update obstacle positions
def update_obstacles(t):
    s0.T[1, 3] = 0.3 - 0.1 * np.sin(0.5 * t)  # Move obstacle 1 sinusoidally
    s1.T[0, 3] -= 0.002  # Move obstacle 2 towards the elbow

# Step function
def step(time):
    update_obstacles(time)

    Te = panda.fkine(panda.q)
    eTep = Te.inv() * Tep
    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

    v, arrived = rtb.p_servo(Te, Tep, 0.5, 0.01)

    Y = 0.01
    Q = np.eye(7 + 6)
    Q[:7, :7] *= Y
    Q[7:, 7:] = (1 / e) * np.eye(6)

    Aeq = np.c_[panda.jacobe(panda.q), np.eye(6)]
    beq = v.reshape((6,))

    Ain = np.zeros((7 + 6, 7 + 6))
    bin = np.zeros(7 + 6)

    ps, pi = 0.05, 0.9
    Ain[:7, :7], bin[:7] = panda.joint_velocity_damper(ps, pi, 7)

    for obstacle in [s0, s1]:
        c_Ain, c_bin = panda.link_collision_damper(obstacle, panda.q[:7], 0.3, 0.05, 1.0,
                                                   start=panda.link_dict["panda_link1"],
                                                   end=panda.link_dict["panda_hand"])
        if c_Ain is not None and c_bin is not None:
            if c_Ain.shape[1] == Ain.shape[1]:
                Ain = np.vstack((Ain, c_Ain))
                bin = np.hstack((bin, c_bin))

    c = np.r_[-panda.jacobm(panda.q).reshape((7,)), np.zeros(6)]
    lb = -np.r_[panda.qdlim[:7], 10 * np.ones(6)]
    ub = np.r_[panda.qdlim[:7], 10 * np.ones(6)]

    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='osqp')
    panda.qd[:7] = qd[:7]

    # Compute distances
    neo_distance_to_goal = np.linalg.norm(Te.t - Tep.t)
    neo_distance_to_obstacle1 = np.linalg.norm(Te.t - s0.T[:3, 3])
    elbow_pose = panda.fkine(panda.q, end=panda.link_dict["panda_link4"])
    neo_distance_to_obstacle2 = np.linalg.norm(elbow_pose.t - s1.T[:3, 3])

    # Compute manipulability
    manipulability = panda.manipulability(np.array(panda.q))

    # Store data
    data.append([time, neo_distance_to_goal, neo_distance_to_obstacle1, neo_distance_to_obstacle2, manipulability])

    env.step(0.01)
    return arrived

def run():
    arrived = False
    time = 0
    while not arrived:
        arrived = step(time)
        time += 0.01

    df = pd.DataFrame(data, columns=["time", "neo_distance_to_goal", "neo_distance_to_obstacle1", "neo_distance_to_obstacle2", "manipulability"])
    df.to_csv("d2_results.csv", index=False)
    print("CSV saved successfully!")

# Run experiment
run()
