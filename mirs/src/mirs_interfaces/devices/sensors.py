from mirs_interfaces.msg import MotorSensorFeedback
from mirs_controller.common.topics import TOPICS
from rclpy.node import Node

class JointSensor(Node):
    def __init__(self,name)-> None:
        super().__init__('Sensor')
        self.name = name
        self.state = MotorSensorFeedback()
        self.state_subscriber = self.create_subscription(MotorSensorFeedback,TOPICS.TOPIC_MOTOR_SENSOR_STATE,self.set_state,1)

    def get_position(self):
        return self.state
    
    def get_velocity(self):
        return self.state
    
    def set_state(self,msg):
        self.state.position = msg.position
        self.state.velocity = msg.velocity
        self.state.torque = msg.torque