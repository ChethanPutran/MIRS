import rclpy
from rclpy.node import Node
from  mirs_interfaces.msg import VoiceState,SytemState,ExecutorState,RecorderState,ExtractorState
from  mirs_interfaces.topics.topics import TOPICS
from  mirs_interfaces.topics.commands import COMMANDS

class SystemState:
    RECORDING = "state_recording" 
    RECORDED = "state_recorded" 
    EXECUTING = "state_executing" 
    EXECUTED = "state_executed" 
    PROCESSING = "state_processing" 
    PROCESSED = "state_processed" 
    DONE = "state_done"
    ERROR ="state_error"
    WARNING="state_warning"
    PAUSE = "state_pause" 
    STOP = "state_stop" 

class System(Node):
    def __init__(self):
        super().__init__("System")
        
        self.recorder = {
            "status":None,
            "error":None,
            "recording":None
        }
        self.extractor = {
            "status":None,
            "error":None,
            "tasks":None
        }
        self.executor = {
            "status":None,
            "error":None,
            "recording":None
        }
        self.state = {
            "status":None,
            "error":None,
            "message":None,
            "command":None,
            "recording":None,
        }
        
        self.cmd = None
        self.publishing_period = 1

        self.task_recorder_listener = self.create_subscription(TOPICS.TOPIC_RECORDING,RecorderState,self.recorder_callback)
        self.task_extractor_listener = self.create_subscription(TOPICS.TOPIC_EXTRACTOR_STATUS,ExtractorState,self.extractor_callback)
        self.task_executor_listener = self.create_subscription(TOPICS.TOPIC_EXECUTOR_STATUS,ExecutorState,self.executor_callback)
        self.voice_listener = self.create_subscription(TOPICS.TOPIC_VOICE_STATE,VoiceState,self.set_cmd)
       
        self.system_state_publisher = self.create_publisher(TOPICS.TOPIC_SYSTEM_STATE,SytemState,1)
        self.timer = self.create_timer(self.publishing_period,self.publish_system_state)
        self.start()

    def recorder_callback(self,msg):
        self.get_logger().info("Recorder callback :" + msg.status)
        self.recording['status'] = msg.status
        self.recording['recording'] = msg.recording 
        self.recording['error'] = msg.error

    def extractor_callback(self,msg):
        self.get_logger().info("Extractor callback :" + msg.status)
        self.extractor['status'] = msg.status
        self.extractor['tasks'] = msg.tasks 
        self.extractor['error'] = msg.error
      
    def executor_callback(self,msg):
        self.get_logger().info("Executor callback :" + msg.status)
        self.extractor['status'] = msg.status
        self.extractor['finished'] = msg.finished 
        self.extractor['error'] = msg.error

    def publish_system_state(self):
        self.get_logger().info("Publishing sys state :" + self.state['status'])
        msg = SytemState()
        msg.status = self.state['status']
        msg.message = self.state['message']
        msg.error = self.state['error']

        self.system_state_publisher.publish(msg)

    def set_cmd(self,msg):
        self.cmd = msg.cmd

    def get_state(self):
        return self.state
    
    def set_state(self,status,recording=None,command=None,error=None,message=None):
        self.state['status'] = status
        self.state['message'] = error
        self.state['error'] = message
        self.state['recording'] = recording
        self.state['command'] = command
    
    def get_status(self):
        return self.state['status']
    
    def get_cmd(self):
        return self.cmd
    
    def exists_recoding(self):
        return not(self.recorder['recording'] == None)
    
    def exists_tasks(self):
        return not(self.executor['tasks'] == None)
    
    def get_recoding(self):
        return self.recorder['recording']
    
    def start(self):
        while not (self.get_cmd() == 'exit'):
            cmd = self.get_cmd() 
            # cmd = input("Enter your command : ")
            self.get_logger().info("Command : "+cmd)
            if not cmd:
                continue

            st = self.get_status()

            if((st == SystemState.RECORDING) or \
                (st == SystemState.PROCESSING) or \
                (st == SystemState.EXECUTING)) :
                msg = f"Can not execute command '{cmd}'. System currently {st} the task."
                self.set_state(SystemState.WARNING,command=COMMANDS.SPEAK,message=msg)
                continue
            
            if cmd == 'record':
                print("Recording...")
                self.set_state(SystemState.RECORDING,command=COMMANDS.START_RECORD)
            elif (cmd == 'process'):

                if self.exists_recoding():
                    self.set_state(SystemState.PROCESSING,command=COMMANDS.START_PROCESSING,recording=self.get_recoding())
                else:
                    msg = "No recording exists. Please record the taks first!"
                    self.set_state(SystemState.WARNING,command=COMMANDS.SPEAK,message=msg)
            elif cmd == 'execute':
                if self.exists_tasks():
                    self.set_state(SystemState.EXECUTING)
                else:
                    if(self.exists_recoding()):
                        msg = "No tasks to execute. Please process the recording first!"
                    else:
                        msg = "No tasks to execute. Please record the taks first!"
                    self.set_state(SystemState.WARNING,command=COMMANDS.SPEAK,message=msg)

                #Event.fire(Event.EVENT_EXECUTE_TASK)
                print("Executing...")
                # self.execute()
                sucess, error = self.robot.execute(self.tasks)
                if (not sucess):
                    self.set_state(SystemState.ERROR,error)
                    print("Error : ",error)
                self.set_state(SystemState.EXECUTED)
                print("Task completed sucessfully :)")
            elif cmd == 'exit':
                self.set_state(SystemState.STOP)
                print("Exiting...")
                break
            else:
                self.set_state(SystemState.ERROR,command=COMMANDS.SPEAK,message="No command found! Try again")

        print("Exiting... Bye...")
        self.set_state(SystemState.WARNING,command=COMMANDS.SPEAK, message="Exiting...")

def main(args=None):
    rclpy.init(args)

    system = System()

    rclpy.spin(system)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
