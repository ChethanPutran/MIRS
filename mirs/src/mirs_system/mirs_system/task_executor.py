#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .conf.topics import TOPICS
from .conf.services import SERVICES
from mirs_interfaces.msg import TaskExecutorState, Task, SystemState


class ExecutorState:
    INACTIVE = 'status_inactive'
    EXECUTING = 'status_executing'
    WAITING = 'status_waiting'
    EXECUTED = 'status_executed'
    FAILURE = 'status_failure'


class TaskExecutor(Node):
    def __init__(self, time_period=1):
        super().__init__('TaskExecutor')
        self.tasks = []
        self.TIME_PERIOD = time_period
        self.state = {
            "status": ExecutorState.INACTIVE,
            "error": None,
            "finished": False
        }
        self.task_feedback_success = True
        self.future = None

        self.timer = self.create_timer(
            self.TIME_PERIOD, self.publish_executor_status)

        # Start task
        self.create_subscription(
            SystemState, TOPICS.TOPIC_SYSTEM_STATE, self.execute_tasks)

        # task executor status publisher
        self.task_executor_status_publisher = self.create_publisher(
            TaskExecutorState, TOPICS.TOPIC_EXECUTOR_STATUS, 1)
        self.task_publisher = self.create_publisher(Task, TOPICS.TOPIC_TASK, 1)

        self.set_state(ExecutorState.WAITING)

    def publish_executor_status(self):
        self.get_logger().info('Task status :' + self.state['status'])
        msg = TaskExecutorState()

        msg.status = self.state['status']
        msg.error = self.state['error']
        msg.finished = self.state['finished']

        self.task_executor_status_publisher.publish(msg)

    def set_state(self, status, error=None, finished=False):
        self.state['status'] = status
        self.state['error'] = error
        self.state['finished'] = finished

    def execute_tasks(self, msg):
        if not (msg.cmd == "EXEC_TASK"):
            return

        tasks = msg.tasks
        self.get_logger().info('Task obtained')

        self.set_state(ExecutorState.EXECUTING)

        # Execute task


def main(args=None):
    rclpy.init(args)

    task_executor = TaskExecutor()

    rclpy.spin(task_executor)

    task_executor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
