#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from topics import TOPICS

class Recorder(Node):
    def __init__(self,time_period=1):
        super().__init__("Recorder")
        self.TIME_PERIOD = time_period
        self.state = "INACTIVE"
        self.recording = None

        # Create publisher
        self.recording_state_publisher = self.create_publisher(String,TOPICS.TOPIC_RECORDER_STATUS,10)
        self.timer = self.create_timer(self.TIME_PERIOD,self.get_recorder_state)

        # Create listener
        self.create_subscription(String,TOPICS.TOPIC_START_RECORDING,self.record)

    def get_recorder_state(self):
        msg = String()
        msg.data = self.state
        self.get_logger().info("Recorder state"+self.state)
        self.recording_state_publisher.publish(msg)

    def record(self):
        #Start record 
        count = 0
        self.state="RECORDING"
        while count<50:
            count+=1
        self.get_logger().info("Recording sucessfull")
        self.state="RECORDED"

    def get_recording(self):
        self.state = "INACTIVE"
        return self.recording
        

    
def main(args=None):
    rclpy.init(args=args)

    # Create node
    recorder_node = Recorder()

    # Run continuosly
    rclpy.spin(recorder_node)

    # Destroy node
    rclpy.shutdown()


if __name__ =="__main__":
    try:
        main()
    except Exception as e:
        print(str(e))