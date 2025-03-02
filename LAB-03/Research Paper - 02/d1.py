import swift
import spatialgeometry as sg
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import csv

# Launch the simulator Swift
env = swift.Swift()
env.launch()

# Create a Panda robot object
panda = rtb.models.Panda()

# Set joint angles to ready configuration
panda.q = panda.qr

# Number of joints in the Panda which we are controlling
n = 7

# Make an obstacle with velocity
s0 = sg.Sphere(radius=0.05, pose=sm.SE3(0.52, 0.4, 0.3))
s0.v = [0, -0.2, 0, 0, 0, 0]

collisions = [s0]

# Make a target
target = sg.Sphere(radius=0.02, pose=sm.SE3(0.6, -0.2, 0.0))

# Add the Panda and shapes to the simulator
env.add(panda)
env.add(s0)
env.add(target)

# Set the desired end-effector pose to the location of the target
Tep = panda.fkine(panda.q)
Tep.A[:3, 3] = target.T[:3, -1]

# CSV file setup
csv_filename = "d1_results.csv"
with open(csv_filename, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Time", "Distance to Obstacle", "Distance to Goal", "Manipulability"])

def step(t):
    # The pose of the Panda's end-effector
    Te = panda.fkine(panda.q)

    # Compute Manipulability
    manipulability = panda.manipulability(np.array(panda.q))


    # Transform from the end-effector to desired pose
    eTep = Te.inv() * Tep

    # Spatial error
    e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

    # Calculate the required end-effector spatial velocity for the robot
    v, arrived = rtb.p_servo(Te, Tep, 0.5, 0.01)

    # Gain term (lambda) for control minimization
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(n + 6)

    # Joint velocity component of Q
    Q[:n, :n] *= Y

    # Slack component of Q
    Q[n:, n:] = (1 / e) * np.eye(6)

    # The equality constraints
    Aeq = np.c_[panda.jacobe(panda.q), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((n + 6, n + 6))
    bin = np.zeros(n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach its limit
    ps = 0.05

    # The influence angle (in radians) in which the velocity damper becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[:n, :n], bin[:n] = panda.joint_velocity_damper(ps, pi, n)

    # For each collision in the scene
    for collision in collisions:
        # Form the velocity damper inequality constraint for each collision object
        c_Ain, c_bin = panda.link_collision_damper(
            collision,
            panda.q[:n],
            0.3,
            0.05,
            1.0,
            start=panda.link_dict["panda_link1"],
            end=panda.link_dict["panda_hand"],
        )

        # If there are any parts of the robot within the influence distance to the collision
        if c_Ain is not None and c_bin is not None:
            # Adjust column count to match Ain
            if c_Ain.shape[1] < Ain.shape[1]:
                pad_width = Ain.shape[1] - c_Ain.shape[1]
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], pad_width))]
            elif c_Ain.shape[1] > Ain.shape[1]:
                pad_width = c_Ain.shape[1] - Ain.shape[1]
                Ain = np.c_[Ain, np.zeros((Ain.shape[0], pad_width))]

            # Stack the inequality constraints
            Ain = np.r_[Ain, c_Ain]
            bin = np.r_[bin, c_bin]

    # Linear component of objective function: the manipulability Jacobian
    c = np.r_[-panda.jacobm(panda.q).reshape((n,)), np.zeros(6)]

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[panda.qdlim[:n], 10 * np.ones(6)]
    ub = np.r_[panda.qdlim[:n], 10 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='osqp')

    # Apply the joint velocities to the Panda
    panda.qd[:n] = qd[:n]

    # Record data to CSV file
    with open(csv_filename, "a", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            t, 
            np.linalg.norm(Te.t - s0.T[:3, -1]), 
            np.linalg.norm(Te.t - Tep.t), 
            manipulability
        ])

    # Step the simulator by 50 ms
    env.step(0.01)

    return arrived

def run():
    arrived = False
    t = 0.0
    while not arrived:
        arrived = step(t)
        t += 0.01

step(0)
run()
print(f"Experiment 1a results saved to {csv_filename}")
