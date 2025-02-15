import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib
import PIL.Image

# Step-1: Load a model of the Franka-Emika Panda robot defined by a URDF file
robot = rtb.models.Panda()
#print(robot)


# Step-2: Forward Kinematics
Te = robot.fkine(robot.qr)  # forward kinematics
#print(Te)

# Step-3: Inverse Kinematics
Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
sol = robot.ik_LM(Tep)         # solve IK
#print(sol)

# Step-4: FK shows that desired end-effector pose was achieved
q_pickup = sol[0]
#print(robot.fkine(q_pickup)) 

# Step-5: Trajectory plot
qt = rtb.jtraj(robot.qr, q_pickup, 50)
#robot.plot(qt.q, backend='pyplot', movie='panda1.gif')

# Step-6: Plot the trajectory in the Swift simulator 
robot.plot(qt.q)