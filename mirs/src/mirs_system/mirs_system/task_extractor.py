#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from topics import TOPICS

class TaskExtractor(Node):
    def __init__(self,time_period = 1):
        super().__init__('TaskExtractor')
        self.tasks = []
        self.TIME_PERIOD = time_period
        self.state = "INACTIVE"
        self.task = None
        self.timer = self.create_timer(1,self.extract)
        self.create_subscription(String,TOPICS.TOPIC_START_TASK_EXTRACTION,self.extract)
        self.task_extraction_status_publisher = self.create_publisher(String,TOPICS.TOPIC_EXTRACTOR_STATUS,1)
        self.timer = self.create_timer(self.TIME_PERIOD,self.get_extractor_status)
        
    def get_extractor_status(self):
        self.get_logger().info('Task status :'+self.state)
        msg = String()
        msg.data = self.state
        self.task_extraction_status_publisher.publish(msg)

    def extract(self,recording):
        #extract task
        success = False
        self.state = "EXTRACTING"
        count = 0
        while count < 50:
            count+=1

        #if successfull
        if success:
            self.state = "EXTRACTED"

        #if failure
        else:
            self.state = "FAILURE"

    def get_task(self):
        if self.state == "EXTRACTED":
            self.state = "INACTIVE"
            return self.task
        return None

def main(args=None):
    rclpy.init(args)

    task_extractor_node = TaskExtractor()

    rclpy.spin(task_extractor_node)


    rclpy.shutdown()

if __name__ == "__main__":
    main()
