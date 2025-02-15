#-------- Example-2---------#
# Display Robot with Swift
import roboticstoolbox as rtb
import swift

# Create swift instance
env =swift.Swift()
env.launch(realtime=True)

# Define the robot model
robot = rtb.models.Panda()
robot.q = robot.qr

robot.qd = [0.1, 0, 0, 0, 0, 0, 0.1]

# Add robot to swift
env.add(robot)

for _ in range(100):
    env.step(0.05)

# Stop the browser tab from closing
env.hold()