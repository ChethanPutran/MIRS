import rclpy
from  rclpy.node import Node
from  mirs_interfaces.msg import VoiceState,SystemState,TaskExecutorState,TaskRecorderState,TaskExtractorState
from  .conf.topics import TOPICS
from  .conf.commands import COMMANDS
import time

class State:
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
        self.sys_executor = {
            "status":None,
            "error":None,
            "recording":None
        }
        self.sys_state = {
            "status":None,
            "error":None,
            "message":None,
            "command":None,
            "recording":None,
        }
        
        self.cmd = ''
        self.publishing_period = 1

        self.create_subscription(TaskRecorderState,TOPICS.TOPIC_RECORDING,self.recorder_callback,1)
        self.create_subscription(TaskExtractorState,TOPICS.TOPIC_EXTRACTOR_STATUS,self.extractor_callback,1)
        self.create_subscription(TaskExecutorState,TOPICS.TOPIC_EXECUTOR_STATUS,self.executor_callback,1)
        self.create_subscription(VoiceState,TOPICS.TOPIC_VOICE_STATE,self.set_cmd,1)
       
        self.system_state_publisher = self.create_publisher(SystemState,TOPICS.TOPIC_SYSTEM_STATE,1)
        self.timer = self.create_timer(self.publishing_period,self.publish_system_state)
        self.start()

    def recorder_callback(self,msg):
        self.get_logger().info("Recorder callback :" + msg.status)
        self.recorder['status'] = msg.status
        self.recorder['recording'] = msg.recording 
        self.recorder['error'] = msg.error

    def extractor_callback(self,msg):
        self.get_logger().info("Extractor callback :" + msg.status)
        self.extractor['status'] = msg.status
        self.extractor['tasks'] = msg.tasks 
        self.extractor['error'] = msg.error
      
    def executor_callback(self,msg):
        self.get_logger().info("Executor callback :" + msg.status)
        self.sys_executor['status'] = msg.status
        self.sys_executor['finished'] = msg.finished 
        self.sys_executor['error'] = msg.error

    def publish_system_state(self):
        self.get_logger().info("Publishing sys state :" + self.state['status'])
        msg = SystemState()
        msg.status = self.sys_state['status']
        msg.message = self.sys_state['message']
        msg.error = self.sys_state['error']

        self.system_state_publisher.publish(msg)

    def set_cmd(self,msg):
        self.cmd = msg.cmd

    def get_state(self):
        return self.sys_state
    
    def set_state(self,status,recording=None,command=None,error=None,message=None):
        self.sys_state['status'] = status
        self.sys_state['message'] = error
        self.sys_state['error'] = message
        self.sys_state['recording'] = recording
        self.sys_state['command'] = command
    
    def get_status(self):
        return self.sys_state['status']
    
    def get_cmd(self):
        return self.cmd
    
    def exists_recoding(self):
        return not(self.recorder['recording'] == None)
    
    def exists_tasks(self):
        return not(self.sys_executor['tasks'] == None)
    
    def get_recoding(self):
        return self.recorder['recording']
    
    def start(self):
        while not (self.get_cmd() == 'exit'):
            self.get_logger().info("Waiting for command...")
            time.sleep(1)
            cmd = self.get_cmd() 
            # cmd = input("Enter your command : ")
            self.get_logger().info("Command : "+cmd)
            if not cmd:
                continue

            st = self.get_status()

            if((st == State.RECORDING) or \
                (st == State.PROCESSING) or \
                (st == State.EXECUTING)) :
                msg = f"Can not execute command '{cmd}'. System currently {st} the task."
                self.set_state(State.WARNING,command=COMMANDS.SPEAK,message=msg)
                continue
            
            if cmd == 'record':
                print("Recording...")
                self.set_state(State.RECORDING,command=COMMANDS.START_RECORD)
            elif (cmd == 'process'):

                if self.exists_recoding():
                    self.set_state(State.PROCESSING,command=COMMANDS.START_PROCESSING,recording=self.get_recoding())
                else:
                    msg = "No recording exists. Please record the taks first!"
                    self.set_state(State.WARNING,command=COMMANDS.SPEAK,message=msg)
            elif cmd == 'execute':
                if self.exists_tasks():
                    self.set_state(State.EXECUTING)
                else:
                    if(self.exists_recoding()):
                        msg = "No tasks to execute. Please process the recording first!"
                    else:
                        msg = "No tasks to execute. Please record the taks first!"
                    self.set_state(State.WARNING,command=COMMANDS.SPEAK,message=msg)

                #Event.fire(Event.EVENT_EXECUTE_TASK)
                print("Executing...")
                # self.execute()
                sucess, error = self.robot.execute(self.tasks)
                if (not sucess):
                    self.set_state(State.ERROR,error)
                    print("Error : ",error)
                self.set_state(State.EXECUTED)
                print("Task completed sucessfully :)")
            elif cmd == 'exit':
                self.set_state(State.STOP)
                print("Exiting...")
                break
            else:
                self.set_state(State.ERROR,command=COMMANDS.SPEAK,message="No command found! Try again")

        print("Exiting... Bye...")
        self.set_state(State.WARNING,command=COMMANDS.SPEAK, message="Exiting...")

def main(args=None):
    rclpy.init(args=args)

    system = System()

    rclpy.spin(system)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
