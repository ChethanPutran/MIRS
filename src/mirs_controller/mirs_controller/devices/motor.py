#!/usr/bin/env python3
from rclpy.node import Node
from mirs_interfaces.msg import MotorState,MotorSensorFeedback
        
class Motor:
    def __init__(self,motor_name,motor_num):
        super().__init__(motor_name)
        self.id = motor_num
        self.name = motor_name
        self.position = 0
        self.min_position = 0
        self.max_position = 0
        self.velocity = 0
        self.max_velocity = 0
        self.acceleration = 0
        self.torque = 0
        self.max_torque = 0
        self.motor_state_publisher = self.create_publisher(MotorState,f"/{motor_name}_state",10)

    # Setters
    def set_state(self,position,velocity,torque=None,acceleration=None):
        motor_state = MotorState()
        motor_state.data.position = position
        motor_state.data.velocity = velocity
        motor_state.data.acceleration = acceleration
        motor_state.data.torque = torque

        # Publish to arduino
        self.motor_state_publisher.publish(motor_state)

    def set_available_torque(self, torque):           
        self.max_torque = torque

    def get_position(self):
        return self.position

    def get_min_position(self):
        return self.min_position

    def get_max_position(self):
        return self.max_position
    
    def get_velocity(self):
        return self.velocity

    def get_max_velocity(self):
        return self.max_velocity

    def get_acceleration(self):
        return self.acceleration

    def get_torque(self):
        return self.torque

    def get_max_torque(self):
        return self.max_torque

    
   