import rclpy
from rclpy.node import Node
from mirs_interfaces.msg import GuiState, SystemState, TaskExecutorState, TaskRecorderState, TaskExtractorState
from .conf.topics import TOPICS
from .conf.commands import COMMANDS
from .conf.states import States


class System(Node):
    def __init__(self):
        super().__init__("System")

        self.state = {
            "status": '',
            "error": '',
            "message": '',
            "command": '',
            "recording": '',
            "tasks": []
        }

        self.pre_cmd = ''
        self.publishing_period = 3  # s

        self.create_subscription(
            TaskRecorderState, TOPICS.TOPIC_RECORDER_STATUS, self.recorder_callback, 1)
        self.create_subscription(
            TaskExtractorState, TOPICS.TOPIC_EXTRACTOR_STATUS, self.extractor_callback, 1)
        self.create_subscription(
            TaskExecutorState, TOPICS.TOPIC_EXECUTOR_STATUS, self.executor_callback, 1)
        # self.create_subscription(
        #     GuiState, TOPICS.TOPIC_GUI_STATE, self.set_cmd, 1)

        self.system_state_publisher = self.create_publisher(
            SystemState, TOPICS.TOPIC_SYSTEM_STATE, 1)
        self.timer = self.create_timer(
            self.publishing_period, self.publish_system_state)

        # self.set_state(States.INACTIVE)
        self.set_state(States.DONE, recording="test")

    def recorder_callback(self, msg):
        self.get_logger().info("Recorder callback :" + msg.status+" :"+(
            msg.message if msg.message else msg.error))
        if msg.status == States.ERROR:
            self.set_state(States.ERROR, error=msg.error)
        elif msg.status == States.RECORDING:
            self.set_state(States.RECORDING, message=msg.message)
        elif msg.status == States.RECORDED:
            self.set_state(States.RECORDED, message=msg.message)
        elif msg.status == States.RECIEVE_RECORING:
            self.set_state(States.DONE, recording=msg.recording)

    def extractor_callback(self, msg):
        self.get_logger().info("Extractor callback :" + msg.status+" :"+(
            msg.message if msg.message else msg.error))
        if msg.status == States.ERROR:
            self.set_state(States.ERROR, error=msg.error)
        elif msg.status == States.EXTRACTING:
            # Extraction started clear recording (no more required)
            self.clear_recording()
            self.set_state(States.EXTRACTING, message=msg.message)
        elif msg.status == States.EXTRACTED:
            self.set_state(States.DONE, tasks=msg.tasks)
            print("Got tasks ", msg.tasks)

    def executor_callback(self, msg):
        self.get_logger().info("Executor callback :" + msg.status+" :"+(
            msg.message if msg.message else msg.error))
        if msg.status == States.ERROR:
            self.set_state(States.ERROR, error=msg.error)
        elif msg.status == States.EXECUTING:
            # Executing started clear tasks (no more required)
            self.clear_tasks()
            self.set_state(States.EXECUTING, message=msg.message)
        elif msg.status == States.EXECUTED:
            self.set_state(States.EXECUTED, message=msg.message)

    def publish_system_state(self):
        # Get input from user
        self.get_command_from_user()

        msg = SystemState()
        msg.status = self.state['status']
        msg.message = self.state['message']
        msg.error = self.state['error']
        msg.recording = self.state['recording']
        msg.tasks = self.state['tasks']
        msg.command = self.state['command']

        log = f"Sys state : {self.state['status']} : {self.state['error'] if self.state['error'] else self.state['message']}"
        self.get_logger().info(log)

        self.system_state_publisher.publish(msg)

        if self.state['command']:
            self.clear_command()

    def clear_command(self):
        self.state['command'] = ''

    def set_command(self, command, message=''):
        self.state['command'] = command

        if message:
            self.state['message'] = message

    def get_command_from_user(self, msg=None):
        error = None

        with open('/home/chethan/Desktop/MajorProject/mirs/src/mirs_system/mirs_system/command.txt', 'r') as f:
            cmd = f.read()

        # or
        # cmd = msg.cmd

        if (not cmd) or (cmd == self.pre_cmd):
            return

        self.get_logger().info("New command :"+cmd)

        # st = self.get_status()

        # if ((st == States.RECORDING) or
        #     (st == States.PROCESSING) or
        #         (st == States.EXECUTING)):
        #     msg = f"Can not execute command '{cmd}'. System currently {st} the task."
        #     self.set_state(States.WARNING, command=COMMANDS.SPEAK, message=msg)
        #     return

        if (self.get_status() == States.DONE):
            if self.exists_tasks() and not (cmd == "execute"):
                self.set_error("Already processed tasks! Ready to execute.")
                return
            elif self.exists_recoding() and not (cmd == "process"):
                self.set_error("Already recorded actions! Ready to process.")
                return

        if 'record' in cmd:
            if cmd == "start_record":
                self.set_command(COMMANDS.START_RECORD)
            elif cmd == "end_record":
                self.set_command(COMMANDS.END_RECORD)
            elif cmd == "get_record":
                self.set_command(COMMANDS.GET_RECORDING)
            else:
                error = "No command found! Try again"

        elif (cmd == 'process'):
            if self.exists_recoding():
                self.set_command(COMMANDS.START_PROCESSING)
            else:
                msg = "No recording exists. Please record the taks first!"
                self.set_state(
                    States.WARNING, command=COMMANDS.SPEAK, message=msg)
        elif cmd == 'execute':
            if self.exists_tasks():
                self.set_command(COMMANDS.START_EXECUTION)
            else:
                if (self.exists_recoding()):
                    msg = "No tasks to execute. Please process the recording first!"
                else:
                    msg = "No tasks to execute. Please record the taks first!"
                self.set_state(
                    States.WARNING, command=COMMANDS.SPEAK, message=msg)

            print("Executing...")
            # self.execute()
            sucess, error = self.robot.execute(self.tasks)
            if (not sucess):
                self.set_state(States.ERROR, error)
                print("Error : ", error)
            self.set_state(States.EXECUTED)
            print("Task completed sucessfully :)")
        elif cmd == 'exit':
            self.set_state(States.WARNING, command=COMMANDS.SPEAK,
                           message="Exiting...")
            self.set_state(States.STOP)
            print("Exiting...")
            return
        else:
            error = "No command found! Try again"

        if error:
            self.set_state(error)

        self.pre_cmd = cmd

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
        self.state['command'] = command

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
