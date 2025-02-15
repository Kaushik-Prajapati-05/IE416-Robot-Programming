#-------- Example-1---------#
import roboticstoolbox as rtb

# Define the robot model
robot = rtb.models.Panda()

# Print the robot model
print(robot)

# Visulise the robot
robot.plot(robot.qr, block=True)