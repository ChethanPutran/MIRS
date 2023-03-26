#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mirs_controller.mirs_controller.topics import TOPICS
from mirs_controller.mirs_controller.msg import MotorState,RobotState
from mirs_controller.mirs_controller.config import JOINT_NAMES,JOINT_SENSOR_NAMES
from mirs_controller.mirs_controller.devices import Motor,Sensor


class Kinematics:
    def __init__(self)-> None:
        pass
    def forward(self):
        pass
    def inverse(self):
        pass


class Dynamics:
    def __init__(self) -> None:
        pass
    def forward(self):
        pass
    def inverse(self):
        pass


class Robot(Node):
    def __init__(self,camera=None,DOF=6)-> None:
        super().__init__("Robot")
        self.joints = []
        self.DOF = 6
        self.links = []
        self.state = []
        self.kinematics = Kinematics()
        self.dynamics = Dynamics()
        self.devices = {}
        self.time = 0
        self.joints = JOINT_NAMES
        self.sensors = JOINT_SENSOR_NAMES
        self.devices = []

        # Initilaize joint motors
        for joint_name in self.joints:
            self.devices[joint_name] = Motor(name=joint_name)

        # Initilaize joint sensors
        for sensor_name in self.sensors:
            self.devices[sensor_name] = Sensor(name=sensor_name)

        # Add camera as device
        if camera:
            self.devices['camera'] = camera

    def set_state(self,state:RobotState):
        self.state = state

    def get_time(self):
        return self.time

    def get_state(self)-> RobotState:
        return self.state
    
    def get_device(self,name):
        return self.devices[name]

    

def main(args=None):
    rclpy.init(args)

    rclpy.shutdown()


if __name__ =="__main__":
    main()