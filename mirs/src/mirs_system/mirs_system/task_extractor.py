#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .conf.topics import TOPICS
from .conf.services import SERVICES
from .conf.states import States
from .conf.commands import COMMANDS
from mirs_interfaces.msg import TaskExtractorState, SystemState, Task
import time
# from .ai.task.extractor import Extractor


class TaskExtractor(Node):
    def __init__(self, time_period=3):
        super().__init__('TaskExtractor')
        self.tasks = []
        self.TIME_PERIOD = time_period
        self.state = {
            'status': '',
            'tasks': [],
            'error': '',
            'message': '',
        }
        # self.task_extractor = Extractor()

        self.create_subscription(
            SystemState, TOPICS.TOPIC_SYSTEM_STATE, self.extract_tasks, 1)

        self.task_extraction_status_publisher = self.create_publisher(
            TaskExtractorState, TOPICS.TOPIC_EXTRACTOR_STATUS, 1)

        self.timer = self.create_timer(
            self.TIME_PERIOD, self.publish_extractor_status)

        self.set_state(States.INACTIVE)

    def set_state(self, status, tasks=[], error='', message=''):
        self.state['status'] = status
        self.state['error'] = error
        self.state['message'] = message

        if tasks:
            self.state['tasks'] = tasks

    def set_error(self, error):
        self.state['error'] = error

    def get_state(self):
        return self.state

    def publish_extractor_status(self):
        self.get_logger().info('Task status :'+self.state['status'])
        msg = TaskExtractorState()
        msg.status = self.state['status']
        msg.tasks = self.state['tasks']
        msg.error = self.state['error']
        msg.message = self.state['message']
        self.task_extraction_status_publisher.publish(msg)

    def is_present_tasks(self):
        return len(self.state['tasks'])

    def clear_tasks(self):
        self.state['tasks'] = []

    def extract_tasks(self, msg):
        if (msg.status == States.DONE) and self.is_present_tasks():
            self.clear_tasks()
            self.set_state(States.INACTIVE)

        self.get_logger().info('Command :'+msg.command)
        if not (msg.command == COMMANDS.START_PROCESSING):
            return
        print("Extracting")
        recording = msg.recording

        self.set_state(States.EXTRACTING)

        # success, task_queue, error = self.task_extractor.extract(recording)

        t1 = Task()
        t1.action = 'pick'
        t2 = Task()
        t2.action = 'move'
        t3 = Task()
        t3.action = 'place'

        success, task_queue, error = True, [t1, t2, t3], None

        self.get_clock().sleep_for(rel_time=time.se)

        # if successfull
        if success:
            self.set_state(States.EXTRACTED, task_queue)
            self.tasks = task_queue

        # if failure
        else:
            self.set_state(States.ERROR, error=error)


def main(args=None):
    rclpy.init(args=args)

    task_extractor_node = TaskExtractor()

    rclpy.spin(task_extractor_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
