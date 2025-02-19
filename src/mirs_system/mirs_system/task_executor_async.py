#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mirs_interfaces.topics.topics import TOPICS
from mirs_interfaces.topics.services import SERVICES
from mirs_interfaces.msg import TaskExecutorState
from mirs_interfaces.srv import ExecTask


class ExecutorState:
    INACTIVE='status_inactive'
    EXECUTING='status_executing'
    WAITING='status_waiting'
    EXECUTED='status_executed'
    FAILURE='status_failure'

class TaskExecutorAsync(Node):
    def __init__(self,time_period = 1):
        super().__init__('TaskExecutorAsync')
        self.tasks = []
        self.TIME_PERIOD = time_period
        self.state = {
            "status":ExecutorState.INACTIVE,
            "error":None,
            "finished":False
        }
        self.task_feedback_success = True
        self.future = None

        # Task executor service
        self.execute_task_client = self.create_client(ExecTask,SERVICES.SERVICE_EXECUTE_TASK)

        self.timer = self.create_timer(self.TIME_PERIOD,self.publish_executor_status)

        # Start task 
        self.create_subscription(String,TOPICS.TOPIC_START_TASK,self.execute_tasks)

        # task executor status publisher
        self.task_executor_status_publisher = self.create_publisher(TaskExecutorState,TOPICS.TOPIC_EXECUTOR_STATUS,1)

        self.state = ExecutorState.WAITING
        while not self.execute_task_client.wait_for_service(timeout_sec=1):
            self.get_logger().info("Task executor service not available! Waiting...")

    
    def publish_executor_status(self):
        self.get_logger().info('Task status :' + self.state['status'])
        msg = TaskExecutorState()

        msg.status = self.state['status']
        msg.error = self.state['error']
        msg.finished = self.state['finished']

        self.task_executor_status_publisher.publish(msg)

    def set_state(self,status,error=None,finished=False):
        self.state['status'] = status
        self.state['error'] = error
        self.state['finished'] = finished

    def execute_tasks(self,msg):
        tasks = msg.tasks
        self.get_logger().info('Task obtained')

        self.set_state(ExecutorState.EXECUTING) 
    
        req = ExecTask.Request()
        req.TASKS = tasks
        self.future = self.execute_task_client.call_async(req)



def main(args=None):
    rclpy.init(args)

    task_extractor_node = TaskExecutorAsync()

    while rclpy.ok():
        rclpy.spin_once(task_extractor_node)

        if(task_extractor_node.future and task_extractor_node.future.done()):
            try:
                res = task_extractor_node.future.result()
                if res.error:
                    raise (res.error)
            except Exception as e:
                task_extractor_node.set_state(ExecutorState.FAILURE,error=str(e))
            else:
                task_extractor_node.set_state(ExecutorState.EXECUTED,finished=True)
            break
        
    task_extractor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
