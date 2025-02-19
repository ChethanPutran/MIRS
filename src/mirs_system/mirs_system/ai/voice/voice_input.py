# Importing neccessary modules
import speech_recognition as sr
from  mirs_interfaces.msg import VoiceState,SystemState
from  mirs_system.conf.topics import TOPICS
from  mirs_system.conf.states import States
from  mirs_system.conf.commands import COMMANDS
import rclpy
from rclpy.node import Node

# Node
class VoiceInput(Node):
    def __init__(self):
        super().__init__("MIRS_Voice_input")
        self.command_queue = []
        self.command_queue_size = 0
        self.command = ''
        self.error = ''
        self.sys_state = ''
        self.sys_command = ''
        self.recognizer = sr.Recognizer()
        self.voice_pub_period = 2 #sec
        self.listening_period = 10 #sec

        self.timer1 = self.create_timer(self.listening_period,self.handle_command)
        self.timer2 = self.create_timer(self.voice_pub_period,self.publish_voice_state)
        self.create_subscription(SystemState,TOPICS.TOPIC_SYSTEM_STATE,self.sys_state_callback,1)
        self.voice_cmd_publisher = self.create_publisher(VoiceState,TOPICS.TOPIC_VOICE_STATE,1)


    def handle_command(self):
        cmd = self.listen()
        self.process_command(cmd)

    
    def sys_state_callback(self,msg:SystemState):
        self.sys_state = msg.status

        if msg.command == COMMANDS.EXIT:
            self.destroy_node()
            rclpy.shutdown()
            exit(0)
        

    def publish_voice_state(self):
        if self.command_queue_size>0:
            self.command_queue_size-=1
            (command,error) = self.command_queue.pop(0) # Get element from front
            self.get_logger().info("Publishing voice state...")
            msg = VoiceState()
            msg.command = command 
            msg.error = error
            self.voice_cmd_publisher.publish(msg)


    def process_command(self,command:str):
        command = command.strip()
        if (not command):
            return
        
        error = ''
    
        self.get_logger().info("New command : "+command+". Sys State :"+self.sys_state)

      
        if('pause' in command):
            command = COMMANDS.PAUSE
        elif((self.sys_state == States.PAUSE) and 'resume' in command):
            command = COMMANDS.RESUME

        elif (self.sys_state == States.EXTRACTING) or (self.sys_state == States.EXECUTING):
            error = f"Can not execute command '{command}' while system processing the recording!"

        elif (self.sys_state == States.RECORDING):
            if ('record' in command) and ('stop' in command):
                command = COMMANDS.END_RECORD
            else:
                error = f"Can not execute the command '{command}' while system recording the task!"
               

        elif (self.sys_state == States.RECORDED):
            if command != "process":
                error = "Already recorded actions! Ready to process."
            else:
                command = COMMANDS.START_PROCESSING
        
        elif (self.sys_state == States.EXTRACTED):
            if command != "execute":
                error = "Already processed tasks! Ready to execute."
            else:
                command = COMMANDS.START_EXECUTION

        elif ('record' in command) and ('start' in command):
            command = COMMANDS.START_RECORD
    
        elif ('process' in command):
            error = "No recording exists. Please record the tasks first!"
            
        elif ('execute' in command):
            error = "No tasks to execute. Please record the tasks first!"

        elif('exit' in command):
            command = COMMANDS.EXIT
        else:
            error = "No command found! Try again."

        if error:
            self.set_voice_state(error=error)
        else:
            self.set_voice_state(command)

    def set_voice_state(self,command='',error=''):
        self.command_queue.append((command,error))
        self.command_queue_size += 1

    def listen(self):
        # It takes microphone input from the user and returns string output
        command = ''
        try:
            with sr.Microphone() as source:
                self.get_logger().info("Listening...")
                audio = self.recognizer.listen(source, timeout=3,phrase_time_limit=3)
                self.get_logger().info("Recognizing...")
                command = self.recognizer.recognize_google(audio)
        except Exception as e:
            self.get_logger().error(str(e))
        return command


def main(args=None):
    try:
        rclpy.init(args=args)

        voice = VoiceInput()

        rclpy.spin(voice)
    except Exception as e:
        print(e)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
