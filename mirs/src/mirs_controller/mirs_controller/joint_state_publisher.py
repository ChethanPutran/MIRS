#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mirs_controller.msg import MotorState,JointState
from mirs_controller.topics import TOPICS
from mirs_controller.robot import Robot 
from mirs_controller.config import JOINT_NAMES,JOINT_SENSOR_NAMES
import numpy as np
import time


class JointStatePublisher(Node):
    def __init__(self,robot:Robot)-> None:
        super().__init__('Motor')
        self.robot = robot
        self.state = JointState(robot.DOF)
        self.previous_joint_states = JointState(robot.DOF)
        self.state_publisher = self.create_publisher(MotorState,TOPICS.TOPIC_JOINT_STATE,1)
        self.state_subscriber = self.create_subscription(MotorState,TOPICS.TOPIC_JOINT_STATE,self.set_state,10)
        self.joint_names = JOINT_NAMES
        self.joint_sensor_names = JOINT_SENSOR_NAMES
        self.current_time = 0
        self.previous_time = 0
        self.motors = []
        self.sensors = []
        for joint_name,sensor_name in enumerate(self.joint_names,self.joint_sensor_names):
            self.motors.append(robot.get_device(joint_name))
            self.sensors.append(robot.get_device(sensor_name))

    def set_vals(self):
        self.current_time = time.time()
        time_diff = (self.robot.get_time() - self.previous_time)
        msg = JointState()
        msg.header.frame_id = "From simulation state data"
        msg.name = [s + self.jointPrefix for s in self.joint_names]
        msg.position = []
        
        for i in range(len(self.sensors)):
            value = self.sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.previousPosition[i]) / time_diff if time_diff > 0 else 0.0)
            self.previousPosition[i] = value
        
        self.previous_time = self.current_time

    def set_state(self,state: JointState):
        self.get_logger().info(f"Got state : {state.to_string()}")
        self.state = state

    def get_state(self)-> MotorState:
        return self.state
    
    def publish(self):
        """Publish the 'joint_states' topic with up to date value."""
        msg = JointState()
        msg.header.stamp = self.robot.get_time()
        msg.header.frame_id = "From simulation state data"
        msg.name = [s + self.jointPrefix for s in self.joint_names]
        msg.position = []
        timeDifference = self.robot.get_time() - self.previousTime
        for i in range(len(self.sensors)):
            value = self.sensors[i].getValue()
            msg.position.append(value)
            msg.velocity.append((value - self.previousPosition[i]) / timeDifference if timeDifference > 0 else 0.0)
            self.previousPosition[i] = value
        msg.effort = [0] * 6
        self.publisher.publish(msg)
        self.last_joint_states = msg
        self.previousTime = self.robot.get_time()
