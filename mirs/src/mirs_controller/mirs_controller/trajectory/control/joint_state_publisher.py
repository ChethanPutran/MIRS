#!/usr/bin/env python3
from rclpy.node import Node
from mirs_interfaces.msg import MotorSensorFeedback,JointState
from mirs_controller.common.topics import TOPICS
from mirs_controller.common.config import JOINT_NAMES,JOINT_SENSOR_NAMES

class JointStatePublisher(Node):
    def __init__(self,publish_period=1)-> None:
        super().__init__('JointStatePublisher')
        self.state = {}
        self.previous_joint_states = {}
        self.publish_period = publish_period

        for i,joint_sensor in enumerate(JOINT_SENSOR_NAMES):
            self.create_subscription(MotorSensorFeedback,joint_sensor+'_feedback',self.set_state)
            self.state[JOINT_NAMES[i]] = {"position":0,"velocity":0,"acceleration":0,"torque":0}
            self.previous_joint_states[JOINT_NAMES[i]] = {"position":0,"velocity":0,"acceleration":0,"torque":0}

        self.state_publisher = self.create_publisher(JointState,TOPICS.TOPIC_JOINT_STATE,1)
        self.current_time = 0
        self.previous_time = 0

        self.timer = self.create_timer(publish_period,self.publish_state)
    
    def set_state(self,msg):
        self.get_logger().info(f"Got state : {msg}")
        self.state[msg.data.name]['position'] = msg.data.position
        self.state[msg.data.name]['velocity'] = msg.data.velocity
        self.state[msg.data.name]['acceleration'] = msg.data.acceleration
        self.state[msg.data.name]['torque'] = msg.data.torque

    def publish_state(self):
        msg = JointState()
        msg.data = self.state
        self.state_publisher.publish(msg)
    