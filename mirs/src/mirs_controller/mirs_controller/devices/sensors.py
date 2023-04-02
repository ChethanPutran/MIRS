from mirs.src.mirs_interfaces.msg.msg import SensorState
from common.topics import TOPICS
from rclpy.node import Node

class JointSensor(Node):
    def __init__(self,name)-> None:
        super().__init__('Sensor')
        self.name = name
        self.state = SensorState()
        self.state_publisher = self.create_publisher(SensorState,TOPICS.TOPIC_SENSOR_STATE,10)
        self.state_subscriber = self.create_subscription(SensorState,TOPICS.TOPIC_SENSOR_STATE,self.set_state)

    def get_position(self):
        return self.state
    
    def get_velocity(self):
        return self.state