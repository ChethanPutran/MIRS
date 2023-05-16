import rclpy
from rclpy.node import Node
from mirs_interfaces.msg import VoiceState, SystemState, TaskExecutorState, TaskRecorderState, TaskExtractorState
from .conf.topics import TOPICS
from .conf.states import States
import uuid
  
class Command:
    def __init__(self,cmd) -> None:
        self.id = uuid.uuid1()
        self.command = cmd

DUMMY_CMD = Command('')
class System(Node):
    def __init__(self):
        super().__init__("System")

        self.state = {
            "status": States.INACTIVE,
            "error": '',
            "message": '',
            "command": DUMMY_CMD,
            "recording": '',
            "tasks": []
        }

        self.pre_cmd = ''
        self.command_queue = []
        self.pre_command_processed = []
        self.publishing_period = 1  # s

        self.create_subscription(
            TaskRecorderState, TOPICS.TOPIC_RECORDER_STATUS, self.recorder_callback, 1)
        self.create_subscription(
            TaskExtractorState, TOPICS.TOPIC_EXTRACTOR_STATUS, self.extractor_callback, 1)
        self.create_subscription(
            TaskExecutorState, TOPICS.TOPIC_EXECUTOR_STATUS, self.executor_callback, 1)
        self.create_subscription( VoiceState, TOPICS.TOPIC_VOICE_STATE, self.set_command, 1)
        # self.create_subscription(
        #     GuiState, TOPICS.TOPIC_GUI_STATE, self.set_cmd, 1)

        self.system_state_publisher = self.create_publisher(SystemState, TOPICS.TOPIC_SYSTEM_STATE, 1)
        self.timer = self.create_timer(
            self.publishing_period, self.publish_system_state)


    def recorder_callback(self, msg:TaskRecorderState):
        self.get_logger().info("Recorder callback :" + msg.status +" :"+(
            msg.mssg if msg.mssg else msg.error))
        if msg.status == States.ERROR:
            self.set_state(States.ERROR, error=msg.error)
        elif msg.status == States.RECORDING:
            self.set_state(States.RECORDING, message=msg.mssg)
        elif msg.status == States.RECORDED:
            self.set_state(States.RECORDED, message=msg.mssg,recording=msg.recording)
            
    def extractor_callback(self, msg):
        self.get_logger().info("Extractor callback :" + msg.status+" :"+(
            msg.mssg if msg.mssg else msg.error))
        if msg.status == States.ERROR:
            self.set_state(States.ERROR, error=msg.error)
        elif msg.status == States.EXTRACTING:
            # Extraction started clear recording (no more required)
            self.clear_recording()
            self.set_state(States.EXTRACTING, message=msg.mssg)
        elif msg.status == States.EXTRACTED:
            self.set_state(States.DONE, tasks=msg.tasks)
            print("Got tasks ", msg.tasks)

    def executor_callback(self, msg):
        self.get_logger().info("Executor callback :" + msg.status+" :"+(
            msg.mssg if msg.mssg else msg.error))
        if msg.status == States.ERROR:
            self.set_state(States.ERROR, error=msg.error)
        elif msg.status == States.EXECUTING:
            # Executing started clear tasks (no more required)
            self.clear_tasks()
            self.set_state(States.EXECUTING, message=msg.mssg)
        elif msg.status == States.EXECUTED:
            self.set_state(States.EXECUTED, message=msg.mssg)

    def publish_system_state(self):
        # Get input from user
        msg = SystemState()
        msg.status = self.state['status']
        msg.mssg = self.state['message']
        msg.error = self.state['error']
        msg.recording = self.state['recording']
        msg.tasks = self.state['tasks']


        msg.command = self.state['command'].command

        log = f"Sys state : {self.state['status']} : {self.state['error'] if self.state['error'] else self.state['message']} Command :{msg.command}"
        self.get_logger().info(log)

        self.system_state_publisher.publish(msg)

        # Clear state data once data is published.
        if self.state['command'].command:
            self.pre_command_processed.append(self.state['command'].id)
            self.clear_data()

    def clear_data(self):
        self.state['message'] = ''
        self.state['error'] = ''
        self.state['command'] = DUMMY_CMD

    def set_command(self, msg:VoiceState):
        command = msg.command

        if not command:
            return
        
        self.command_queue.append(command)
        self.get_logger().info("CMD : "+command)

        # Check wether previous command processed or not
        if((not self.state['command'].command) or (self.pre_command_processed[-1]==self.state['command'].id)):
            #Previos command processed continue with new one
            command = self.command_queue.pop()
            self.get_logger().info("SET CMD : "+command)
            self.state['command'] = Command(command)

    def get_state(self):
        return self.state

    def clear_recording(self):
        self.state['recording'] = ''

    def clear_tasks(self):
        self.state['tasks'] = []

    def set_state(self, status, recording='', command='', error='', message='', tasks=[]):
        self.state['status'] = status
        self.state['message'] = message
        self.state['error'] = error
        self.set_command(command)

        if recording:
            self.set_recording(recording)

        if tasks:
            self.set_tasks(tasks)  

    def set_recording(self, recording):
        self.state['recording'] = recording

    def set_tasks(self, tasks):
        self.state['tasks'] = tasks

    def get_status(self):
        return self.state['status']

    def set_error(self, error):
        self.state['error'] = error

    def exists_recoding(self):
        return not (self.state['recording'] == None)

    def exists_tasks(self):
        return len(self.state['tasks']) > 0

    def get_recoding(self):
        return self.state['recording']


def main(args=None):
    rclpy.init(args=args)

    system = System()

    rclpy.spin(system)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
