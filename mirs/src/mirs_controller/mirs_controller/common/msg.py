#!/usr/bin/env python3
class SensorState:
    def __init__(self):
        self.value = 0.0
    def get_value(self):
        return self.value

class MotorState:
    def __init__(self):
        self.angle = 0.0
        self.speed = 0.0
        self.torque = 0.0

    def set_angle(self,angle):
        self.angle = angle

    def get_angle(self):
        return self.angle
    
    def set_speed(self,speed):
        self.speed = speed

    def set_torque(self,torque):
        self.torque = torque

class RobotState:
    def __init__(self,x=0.0,y=0,z=0.0,
                 linear_velocity_x=0.0,
                 linear_velocity_y=0.0,
                 linear_velocity_z=0.0,
                 linear_acceleration_x=0.0,
                 linear_acceleration_y=0.0,
                 linear_acceleration_z=0.0,
                 roll=0.0,
                 pitch=0.0,
                 yaw=0.0,
                 angular_velocity_x=0.0,
                 angular_velocity_y=0.0,
                 angular_velocity_z=0.0,
                 angular_acceleration_x=0.0,
                 angular_acceleration_y=0.0,
                 angular_acceleration_z=0.0):
        self.x = x 
        self.y = y
        self.linear_velocity_x = linear_velocity_x
        self.linear_velocity_y = linear_velocity_y
        self.linear_velocity_z = linear_velocity_z
        self.linear_acceleration_x = linear_acceleration_x
        self.linear_acceleration_y = linear_acceleration_y
        self.linear_acceleration_z = linear_acceleration_z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.angular_velocity_x = angular_velocity_x
        self.angular_velocity_y = angular_velocity_y
        self.angular_velocity_z = angular_velocity_z
        self.angular_acceleration_x = angular_acceleration_x
        self.angular_acceleration_y = angular_acceleration_y
        self.angular_acceleration_z = angular_acceleration_z

    def set_pose(self,roll:float,pitch:float,yaw:float):
        self.roll = roll
        self.pitch = pitch
        self.roll = yaw

    def set_position(self,x:float,y:float,z:float):
        self.x = x
        self.y = y
        self.z = z
    
    def set_linear_velocities(self,vel_x:float,vel_y:float,vel_z:float):
        self.linear_velocity_x = vel_x
        self.linear_velocity_y = vel_y
        self.linear_velocity_z = vel_z
    
    def set_linear_accelerations(self,a_x,a_y,a_z):
        self.linear_acceleration_x = a_x
        self.linear_acceleration_y = a_y
        self.linear_acceleration_z = a_z

    def set_angular_velocities(self,w_x,w_y,w_z):
        self.angular_velocity_x = w_x
        self.angular_velocity_y = w_y
        self.angular_velocity_z = w_z

    def set_linear_accelerations(self,alpha_x,alpha_y,alpha_z):
        self.angular_velocity_x = alpha_x
        self.angular_velocity_y = alpha_y
        self.angular_velocity_z = alpha_z

class JointState:
    def __init__(self,joints=6) -> None:
        self.state = []

    def set_state(self,state:MotorState):
        self.state = state

    def get_state(self)->MotorState:
        return self.state
    
    def to_string(self)->str:
        return self.state