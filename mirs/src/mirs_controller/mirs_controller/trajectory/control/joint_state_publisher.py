#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mirs_interfaces.msg import MotorSensorFeedback,JointState
from mirs_interfaces.topics.topics import TOPICS
from mirs_controller.common.config import JOINT_NAMES,JOINT_SENSOR_NAMES
from mirs_interfaces.devices.motor import Motor
from mirs_interfaces.hardware.hardware import Hardware

class JointStatePublisher(Node):
    def __init__(self,publish_period=1)-> None:
        super().__init__('JointStatePublisher')
        self.previous_joint_states = {}
        self.publish_period = publish_period
        self.state_publisher = self.create_publisher(JointState,TOPICS.TOPIC_JOINT_STATE,1)
        self.current_time = 0
        self.previous_time = 0
        self.n_joints = len(JOINT_SENSOR_NAMES)
        self.state = {
            "theta":[0] * self.n_joints,
            "theta_dot":[0] * self.n_joints,
            "theta_dotdot":[0] * self.n_joints,
            "torque":[0] * self.n_joints,
        }
        self.hardware = Hardware()
        self.timer = self.create_timer(publish_period,self.publish_state)

        # self.motors = [0] * self.n_joints
        # # Set motors
        # for i,motor_name in enumerate(JOINT_NAMES[:self.n_joints]):
        #     self.motors[i] = Motor(motor_name,JOINT_NAMES[motor_name])
    
    def set_state(self,position,velocity,acceleration=[],torque=[]):
        self.state['theta'][:]= position
        self.state['theta_dot'][:] = velocity

        if len(acceleration)>0:
            self.state['theta_dotdot'][:] = acceleration
        if len(torque)>0:
            self.state['torque'][:] = torque

    def publish_state(self):
        self.get_motor_sensor_data()
        self.get_logger().info("Publishing joint state...")
        msg = JointState()
        msg.data = self.state
        self.state_publisher.publish(msg)

    def get_motor_sensor_data(self):
        self.set_state(self.hardware.get_state())
        
    
def main(args=None):
    rclpy.init(args=args)
    
    jsp = JointStatePublisher()

    rclpy.spin(jsp)

    jsp.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()