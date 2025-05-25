
URDF_FILE = r"C:\\Chethan\\Technologies\\Robotics\\Major_Project\\mirs\\src\\mirs_description\\urdf\\robot.xacro"

import pybullet as p
import pybullet_data
import time

# Connect to PyBullet GUI
p.connect(p.GUI)

# Optional: Set the search path to find standard models
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane for reference
#plane_id = p.loadURDF("plane.urdf")

# Load your robot URDF file
# Replace 'your_robot.urdf' with the full path if not in the current folder
robot_id = p.loadURDF(URDF_FILE, useFixedBase=True)

# Set gravity and simulation timestep
p.setGravity(0, 0, -9.81)

# Run simulation to keep the window open
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)