import rclpy
from rclpy.node import Node
from mirs_interfaces.msg import VoiceState, SystemState, TaskExecutorState, TaskRecorderState, TaskExtractorState
from .conf.topics import TOPICS
from .conf.states import States
from .conf.commands import COMMANDS
import uuid
  
class Command:
    def __init__(self,cmd) -> None:
        self.id = str(uuid.uuid1())
        self.command = cmd

class System(Node):
    def __init__(self):
        super().__init__("System")

        self.state = {
            "status": States.RECORDED,
            "error": '',
            "message": '',
            "command": '',
            "recording": [r"C:\Chethan\Mechanical\projects\Major_Project\software\MIRS\mirs\src\mirs_system\mirs_system\ai\vision\camera\recordings\left_1684305050.avi",
                         r"C:\Chethan\Mechanical\projects\Major_Project\software\MIRS\mirs\src\mirs_system\mirs_system\ai\vision\camera\recordings\right_1684305050.avi"],
            "tasks": []
        }
        self.state_queue = []

        self.pre_cmd = ''
        self.command_queue = []
        self.pre_command_processed = []
        self.publishing_period =2 # s

        self.create_subscription(
            TaskRecorderState, TOPICS.TOPIC_RECORDER_STATUS, self.recorder_callback, 1)
        self.create_subscription(
            TaskExtractorState, TOPICS.TOPIC_EXTRACTOR_STATUS, self.extractor_callback, 1)
        self.create_subscription(
            TaskExecutorState, TOPICS.TOPIC_EXECUTOR_STATUS, self.executor_callback, 1)
        self.create_subscription(VoiceState, TOPICS.TOPIC_VOICE_STATE, self.voice_callback, 1)
        # self.create_subscription(
        #     GuiState, TOPICS.TOPIC_GUI_STATE, self.set_cmd, 1)

        self.system_state_publisher = self.create_publisher(SystemState, TOPICS.TOPIC_SYSTEM_STATE, 1)
        self.timer = self.create_timer(
            self.publishing_period, self.publish_system_state)


    def recorder_callback(self, msg:TaskRecorderState):
        if msg.command_id:
            self.get_logger().info("CMD id :"+msg.command_id)
            self.pre_command_processed.append(msg.command_id)

        self.get_logger().info("Recorder callback :" + msg.status +" :"+(
            msg.mssg if msg.mssg else msg.error))
        if msg.status == States.ERROR:
            self.set_state(States.ERROR, error=msg.error)
        elif msg.status == States.RECORDING:
            self.set_state(States.RECORDING, message=msg.mssg)
        elif msg.status == States.RECORDED:
            self.set_state(States.RECORDED, message=msg.mssg,recording=msg.recording)
        elif msg.mssg:
            self.set_state(self.get_status(), message=msg.mssg)
            
    def extractor_callback(self, msg:TaskExtractorState):
        if msg.command_id:
            self.pre_command_processed.append(msg.command_id)

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
        elif msg.mssg:
            self.set_state(self.get_status(), message=msg.mssg)

    def executor_callback(self, msg:TaskExecutorState):
        if msg.command_id:
            self.pre_command_processed.append(msg.command_id)

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
        elif msg.mssg:
            self.set_state(message=msg.mssg)

    def voice_callback(self,msg:VoiceState):
        command = msg.command
        message = msg.message
        error = msg.error

        if error:
            self.set_state(error=error)
        else:
            self.set_state(message=message)

        if not command:
            return
        
        self.command_queue.append(command)
        self.get_logger().info("Got command : "+command)

    def is_previous_command_fullfilled(self,cmd_id):
        if len(self.pre_command_processed)>0:
            if self.pre_command_processed[-1]==cmd_id:
                return True
        return False

    def publish_system_state(self):
        # Check wether previous command processed or not 
        # if processed then continue with new command else publish old command itself
        fullfilled = None

        if self.state['command']:
            fullfilled = self.is_previous_command_fullfilled(self.state['command'].id)

        if(len(self.command_queue)>0):
            # No previous command present
            if not self.state['command']:
                command = self.command_queue.pop(0)
                self.get_logger().info("Setting first command : "+command)
                self.state['command'] = Command(command)

            #if previous command processed continue with new one
            elif self.state['command'] and fullfilled:
                command = self.command_queue.pop(0)
                self.get_logger().info("Setting new command : "+command)
                self.state['command'] = Command(command)
        
        else:
            # No new command clear old command if processed
            if fullfilled:
                self.get_logger().info("Clearing old command...")
                self.state['command'] = ''


        if len(self.state_queue)>0:
            #Update state
            self.get_logger().info("Updating new state...")
            state = self.state_queue.pop(0)
            self.state['status'],self.state['recording'],self.state['error'],self.state['message'],self.state['tasks'] = state

        # Get input from user
        msg = SystemState()
        msg.status = self.state['status']
        msg.mssg = self.state['message']
        msg.error = self.state['error']
        msg.recording = self.state['recording']
        msg.tasks = self.state['tasks']

        if  self.state['command']:
            msg.command = self.state['command'].command
            msg.command_id = self.state['command'].id
        

        log = f"Sys state : {self.state['status']} : {self.state['error'] if self.state['error'] else self.state['message']} Command :{msg.command}"
        self.get_logger().info(log)

        self.system_state_publisher.publish(msg)

        if msg.command == COMMANDS.EXIT:
            self.destroy_node()
            rclpy.shutdown()
            exit(0)
        self.clear_data()


    def clear_data(self):
        if self.state['status'] == States.ERROR:
            self.state['status'] = States.ACTIVE
        self.state['message'] = ''
        self.state['error'] = ''


    def set_state(self, status='', recording=[], error='', message='', tasks=[]):
        if not status:
            self.state_queue.append((self.get_status(),recording,error,message,tasks))
        else:
            self.state_queue.append((status,recording,error,message,tasks))

    def get_status(self):
        return self.state['status']


def main(args=None):
    # try:
    rclpy.init(args=args)

    system = System()

    rclpy.spin(system)
    # except Exception as e:
    #     print(e)
    # finally:
    rclpy.shutdown()


if __name__ == "__main__":
    main()
