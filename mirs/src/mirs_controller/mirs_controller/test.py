import cv2
import sys
import tempfile
import ikpy
from ikpy.chain import Chain
from ikpy.urdf.URDF import get_chain_from_joints
import math
from controller import Supervisor

IKPY_MAX_ITERATIONS = 4

supervisor = Supervisor()
time_step = int(4 * supervisor.getBasicTimeStep())


# Create the arm chain from the URDF
file_name = 'test.urdf'
with open('test.urdf','wb') as file:
    file.write(supervisor.getUrdf().encode('utf-8'))

arm_chain = Chain.from_urdf_file(
                        file_name,
                        active_links_mask=[
                                            False,
                                            True,
                                            True,
                                            True,
                                            True,
                                            True,
                                            False,
                                            False
                                            ])

#arm_chain.active_links_mask[0] = False

arm_motors = []
gripper_motors = []

for link in arm_chain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        
        if not('A motor' in link.name):
            motor.setVelocity(1.0)
            
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(time_step)
        arm_motors.append(motor)
        
    elif ('finger_middle' in link.name) or ('Gripper' in link.name):
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(time_step)
        gripper_motors.append(motor)
        
        
arm_motors[1].setVelocity(0.5)
arm_motors[2].setVelocity(0.5)
arm_motors[3].setVelocity(0.3)

arm_sensors = []
gripper_sensors = []

for motor in arm_motors:
    sensor = motor.getPositionSensor()
    sensor.enable(time_step)
    arm_sensors.append(sensor)
    
for motor in gripper_motors:
    sensor = motor.getPositionSensor()
    sensor.enable(time_step)
    gripper_sensors.append(sensor)
    
arm_values = [sensor.getValue() for sensor in arm_sensors]
print("arm_values ",arm_values)


camera = supervisor.getDevice('camera')
arm = supervisor.getSelf()
box = supervisor.getFromDef('box1')
camera.enable(time_step)


# Main control loop
while supervisor.step(time_step) != -1:
    arm_position = arm.getPosition()
    box_position = box.getPosition()
    
    print("Arm Position : ",arm_position)

    x = 1
    y = 1.3
    z = 0

    initial_position = [0] + [sensor.getValue() for sensor in arm_sensors[1:]]
    print("Initial position :",initial_position)
    
    ik_results = (arm_chain,[x, y, z],initial_position)
    
    print("Ik Results :",ik_results)
    break
    # Recalculate the inverse kinematics of the arm if necessary.
    #position = arm_chain.forward_kinematics(ik_results)
    
    #print("Position",position)
    # squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
    # if math.sqrt(squared_distance) > 0.03:
        # ikResults = armChain.inverse_kinematics([x, y, z])


    # for i in range(1,5):
        # motors[i].setPosition(ikResults[i])

    

