import rclpy
from rclpy.node import Node
from .conf.topics import TOPICS
from .conf.states import States
from .conf.commands import COMMANDS
from mirs_interfaces.msg import TaskExtractorState, SystemState,Tasks,Task
from .ai.extractor import Extractor


class TaskExtractor(Node):
    def __init__(self, time_period=1):
        super().__init__('TaskExtractor')
        self.TIME_PERIOD = time_period
        self.extractor_commands = [COMMANDS.START_PROCESSING,COMMANDS.PAUSE,COMMANDS.RESUME,COMMANDS.EXIT]
        self.state = {
            'status': States.INACTIVE,
            'tasks': [],
            'error': '',
            'message': '',
            'command_id':''
        }
        self.recording = []
        self.task_extractor = Extractor()

        self.create_subscription(
            SystemState, TOPICS.TOPIC_SYSTEM_STATE, self.sys_state_callback, 1)

        self.task_extraction_status_publisher = self.create_publisher(
            TaskExtractorState, TOPICS.TOPIC_EXTRACTOR_STATUS, 1)

        self.timer = self.create_timer(
            self.TIME_PERIOD, self.publish_extractor_status)


    def set_state(self, status, tasks=[], error='', message='',command_id=''):
        self.state['status'] = status
        self.state['error'] = error
        self.state['message'] = message
        self.state['command_id'] = command_id

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
        msg.mssg = self.state['message']
        msg.command_id = self.state['command_id']
        self.task_extraction_status_publisher.publish(msg)
        self.clear_data()

    def clear_data(self):
        if self.state['status'] == States.ERROR:
            self.set_state(States.ACTIVE)
        self.state['error'] = ''
        self.state['message'] = ''

    def is_present_tasks(self):
        return len(self.state['tasks'])

    def clear_tasks(self):
        self.state['tasks'].clear()

    def check_recordings_exist(self):
        return len(self.recording)>0

    def sys_state_callback(self, msg:SystemState):
        if msg.command not in self.extractor_commands:
            return
        
        if( msg.command == COMMANDS.START_PROCESSING) and (self.state['status'] == States.EXTRACTING):
            self.set_state(States.EXTRACTING,message="System is already extracting the tasks! Kindly wait for some minutes.",command_id=msg.command_id)
        
        elif msg.command == COMMANDS.START_PROCESSING:
            if len(msg.recording)>0:
                self.recording = msg.recording
                # Start processing
                self.set_state(States.EXTRACTING,message="Task extraction processed has been  started.",command_id=msg.command_id)
                self.get_logger().info('Extraction started...')

                success, task_queue, error = self.task_extractor.extract(self.recording)

                tasks_msg = Tasks()

                for task in task_queue:
                    tsk = Task()
                    tsk.action = task.action
                    tsk.object = task.object
                    tsk.initial_theta = task.initial_theta
                    tsk.initial_pos = task.initial_pos
                    tsk.final_pos = task.final_pos
                    tsk.final_theta = task.final_theta
                    tasks_msg.tasks.append(tsk)


                # if successfull
                if success:
                    self.set_state(States.EXTRACTED,tasks= tasks_msg,command_id=msg.command_id,message="Task sucessfully extracted.")

                # if failure
                else:
                    self.set_state(States.ERROR, error=error,command_id=msg.command_id)
                    self.clear_tasks()
            else:
                self.set_state(States.ERROR,error="No recording found to process!",command_id=msg.command_id)

        elif msg.command == COMMANDS.EXIT:
            self.destroy_node()
            rclpy.shutdown()
            exit(0)
        else:
            # Implement pause 
            self.get_logger().info("Pausing...")


def main(args=None):
    rclpy.init(args=args)

    task_extractor_node = TaskExtractor()

    rclpy.spin(task_extractor_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
