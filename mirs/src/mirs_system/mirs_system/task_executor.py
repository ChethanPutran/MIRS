#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from topics import TOPICS

class TaskExecutor(Node):
    def __init__(self,time_period = 1):
        super().__init__('TaskExecutor')
        self.tasks = []
        self.TIME_PERIOD = time_period
        self.state = "INACTIVE"
        self.task = None
        self.timer = self.create_timer(1,self.extract)
        self.create_subscription(String,TOPICS.TOPIC_START_TASK,self.execute)
        self.task_executor_status_publisher = self.create_publisher(String,TOPICS.TOPIC_EXECUTOR_STATUS,1)
        self.timer = self.create_timer(self.TIME_PERIOD,self.get_executor_status)
        
    def get_executor_status(self):
        self.get_logger().info('Task status :'+self.state)
        msg = String()
        msg.data = self.state
        self.task_executor_status_publisher.publish(msg)

    def execute(self,task):
        self.get_logger().info('Task obtained :'+task)
        #extract task
        success = False
        self.state = "EXECUTING"
        count = 0
        while count < 50:
            count+=1

        #if successfull
        if success:
            self.state = "EXECUTED"

        #if failure
        else:
            self.state = "FAILURE"


def main(args=None):
    rclpy.init(args)

    task_extractor_node = TaskExecutor()

    rclpy.spin(task_extractor_node)


    rclpy.shutdown()

if __name__ == "__main__":
    main()
