import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import qpsolvers as qp
import numpy as np
import math
import csv
import time

def step_robot(r: rtb.ERobot, Tep, csv_writer, start_time):
    wTe = r.fkine(r.q)
    eTep = np.linalg.inv(wTe) @ Tep

    # Spatial error
    et = np.sum(np.abs(eTep[:3, -1]))

    # Gain term (lambda) for control minimisation
    Y = 0.01

    # Quadratic component of objective function
    Q = np.eye(r.n + 6)
    Q[: r.n, : r.n] *= Y
    Q[:3, :3] *= 1.0 / et
    Q[r.n :, r.n :] = (1.0 / et) * np.eye(6)

    v, _ = rtb.p_servo(wTe, Tep, 1.5)
    v[3:] *= 1.3

    # The equality constraints
    Aeq = np.c_[r.jacobe(r.q), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((r.n + 6, r.n + 6))
    bin = np.zeros(r.n + 6)
    ps = 0.1
    pi = 0.9
    Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

    # Linear component of objective function: the manipulability Jacobian
    c = np.concatenate((np.zeros(3), -r.jacobm(start=r.links[5]).reshape((r.n - 3,)), np.zeros(6)))
    
    kε = 0.5
    bTe = r.fkine(r.q, include_base=False).A
    θε = math.atan2(bTe[1, -1], bTe[0, -1])
    ε = kε * θε
    c[0] = -ε

    # Bounds on joint velocity and slack variable
    lb = -np.r_[r.qdlim[: r.n], 10 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n], 10 * np.ones(6)]

    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="osqp")
    qd = qd[: r.n]

    if et > 0.5:
        qd *= 0.7 / et
    else:
        qd *= 1.4
    
    # Compute manipulability
    manipulability = r.manipulability(r.q, method="yoshikawa")
    
    # Get the elapsed time
    elapsed_time = time.time() - start_time
    
    # Log data to CSV
    csv_writer.writerow([elapsed_time, manipulability, qd.tolist()])
    
    if et < 0.02:
        return True, qd
    else:
        return False, qd

# Initialize Swift environment
env = swift.Swift()
env.launch(realtime=True)
ax_goal = sg.Axes(0.1)
env.add(ax_goal)

frankie = rtb.models.FrankieOmni()
frankie.q = frankie.qr
env.add(frankie)

arrived = False
dt = 0.025

# Behind
env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])
# Set custom (x, y, z) goal position
x_goal = -4  # Set desired x-coordinate
y_goal = 0 # Set desired y-coordinate
z_goal = 0  # Set desired z-coordinate

# Define the transformation matrix for the goal
wTep = sm.SE3(x_goal, y_goal, z_goal)  # Translation to (x, y, z)
ax_goal.T = wTep  # Update goal visualization

ax_goal.T = wTep
env.step()

# Open CSV file to log data
with open("result_0_m4_0.csv", "w", newline="") as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(["Time (s)", "Manipulability", "Joint Velocities"])
    start_time = time.time()
    
    while not arrived:
        arrived, frankie.qd = step_robot(frankie, wTep.A, csv_writer, start_time)
        env.step(dt)
        
        # Reset bases
        base_new = frankie.fkine(frankie.q, end=frankie.links[3]).A
        frankie._T = base_new
        frankie.q[:3] = 0
    
    csv_writer.writerow(["Final", "Position", frankie.q.tolist()])

env.hold()