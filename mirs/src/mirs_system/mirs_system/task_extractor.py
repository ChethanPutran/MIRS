#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mirs_interfaces.topics.topics import TOPICS
from mirs_interfaces.topics.services import SERVICES
from mirs_interfaces.msg import TaskExtractorState 
from .ai.task.extractor import Extractor

class ExtractorState:
    INACTIVE = 'inactive'
    EXTRACTED = 'extracted'
    EXTRACTING = 'extracting'
    FAILURE = 'failure'

class TaskExtractor(Node):
    def __init__(self,time_period = 1):
        super().__init__('TaskExtractor')
        self.tasks = []
        self.TIME_PERIOD = time_period
        self.state = {
            'status':'',
            'tasks':[],
            'error':None
        }
        self.task_extractor = Extractor()

        self.create_subscription(String,TOPICS.TOPIC_START_TASK_EXTRACTION,self.extract_tasks)
        self.task_extraction_status_publisher = self.create_publisher(TaskExtractorState,TOPICS.TOPIC_EXTRACTOR_STATUS,1)
        self.timer = self.create_timer(self.TIME_PERIOD,self.publish_extractor_status)
    
    def set_state(self,status,tasks=[],error=None):
        self.state['status'] = status
        self.state['tasks'] = tasks
        self.state['error'] = error

    def get_state(self):
        return self.state

    def publish_extractor_status(self):
        self.get_logger().info('Task status :'+self.state['status'])
        msg = TaskExtractorState()
        msg.status = self.state['status']
        msg.tasks = self.state['tasks']
        msg.error = self.state['error']
        self.task_extraction_status_publisher.publish(msg)

    def extract_tasks(self,msg):
        recording  = msg.recording

        self.set_state(ExtractorState.EXTRACTING)

        success, task_queue, error = self.task_extractor.extract(recording)

        #if successfull
        if success:
            self.set_state(ExtractorState.EXTRACTED,task_queue)
            self.tasks = task_queue

        #if failure
        else:
            self.set_state(ExtractorState.FAILURE,error=error)


def main(args=None):
    rclpy.init(args)

    task_extractor_node = TaskExtractor()

    rclpy.spin(task_extractor_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
