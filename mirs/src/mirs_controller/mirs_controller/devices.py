#!/usr/bin/env python3
from rclpy.node import Node
from mirs_controller.mirs_controller.msg import MotorState
from mirs_controller.mirs_controller.topics import TOPICS
from mirs_controller.mirs_controller.msg import SensorState

class Sensor(Node):
    def __init__(self,name)-> None:
        super().__init__('Sensor')
        self.name = name
        self.state = SensorState()
        self.state_publisher = self.create_publisher(SensorState,TOPICS.TOPIC_SENSOR_STATE,10)
        self.state_subscriber = self.create_subscription(SensorState,TOPICS.TOPIC_SENSOR_STATE,self.set_state)

    def set_state(self,state: SensorState):
        self.state = state

    def get_state(self)-> SensorState:
        return self.state
    
    
class Motor(Node):
    def __init__(self,name)-> None:
        super().__init__('Motor')
        self.name = name
        self.state = MotorState()
        self.state_publisher = self.create_publisher(MotorState,TOPICS.TOPIC_MOTOR_STATE,10)
        self.state_subscriber = self.create_subscription(MotorState,TOPICS.TOPIC_MOTOR_STATE,self.set_state)

    def set_state(self,state: MotorState):
        self.state = state

    def get_state(self)-> MotorState:
        return self.state
    
    

