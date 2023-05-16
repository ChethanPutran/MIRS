# Importing neccessary modules
import datetime
import speech_recognition as sr
import pyttsx3 as ts
import os
from  mirs_interfaces.msg import VoiceState,SystemState
from  mirs_system.conf.topics import TOPICS
from  mirs_system.conf.states import States
from  mirs_system.conf.commands import COMMANDS
import rclpy
from rclpy.node import Node
import time

# Node
class Voice(Node):
    def __init__(self):
        super().__init__("MIRS_Voice")
        self.pre_cmd = ''
        self.error = ''
        self.message = ''
        self.engine = ts.init()
        self.recognizer = sr.Recognizer()
        # self.recognizer.pause_threshold = 1
        self.START = True
        self.listening_period = 10 #sec

        # Rate
        self.engine.setProperty('rate', 150)

        # Volume 
        self.volume = self.engine.getProperty('volume')
        self.engine.setProperty('volume',1.0)

    
        # Initializing voice for Bot
        self.voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', self.voices[0].id)
        self.pause = None
        
        self.wish()

        self.timer = self.create_timer(self.listening_period,self.handle_command)
        self.create_subscription(SystemState,TOPICS.TOPIC_SYSTEM_STATE,self.sys_state_callback,1)
        self.voice_cmd_publisher = self.create_publisher(VoiceState,TOPICS.TOPIC_VOICE_STATE,1)



    def handle_command(self):
        cmd = self.listen()
        self.process_command(cmd)
        

    def publish_command(self,command):
        msg = VoiceState()
        msg.command= command 
        self.voice_cmd_publisher.publish(msg)


    def sys_state_callback(self,msg:SystemState):
        self.sys_state = msg.status
        if (not msg.error) and (not  msg.mssg):
            return
        
        data = msg.mssg
        tune = 'normal'

        if msg.error:
            tune = 'error'
            data = msg.error
        
        self.get_logger().info('Got sys state :'+msg.status+" Data : "+data)
        self.speak(data,tune=tune)


    def speak(self,audio,tune='normal'):
        if tune == "error":
            audio = "Error! " + audio
        elif tune == "warn":
            audio = "Warning! " + audio
        self.engine.say(audio)
        self.engine.runAndWait()


    def wish(self):
        hour = int(datetime.datetime.now().hour)
        # self.speak("Hi Sir, Welcome to MIRS. MIRS stands for multifunctional intelligent robotic arm \
        #            developed by Mr. Chethan, Mr. Gautham, Mr. Ashish, Mr. Ashwin. Nice to meet you")
        if (hour < 12):
            self.speak("Hi Sir, Good Morning")
        elif (hour >= 12 and hour < 16):
            self.speak("Hi Sir, Good Afternoon")
        elif (hour >= 16 and hour < 19):
            self.speak("Hi Sir, Good Evening")
        else:
            self.speak("Hi Sir, Good Night")


    def process_command(self,command:str):
        command = command.strip()
        if (not command):
            return
    
        self.get_logger().info("New command : "+command)

        if (self.sys_state == States.EXTRACTING) or (self.sys_state == States.EXECUTING):
            msg = f"Can not execute command '{command}'. System currently {self.sys_state} the task."
            self.speak(msg,'warn')

        elif (self.sys_state == States.RECORDED):
            if command != "process":
                self.speak("Already recorded actions! Ready to process.",'error')
            else:
                self.publish_command(COMMANDS.START_PROCESSING)
        
        elif (self.sys_state == States.EXTRACTED):
            if command != "execute":
                self.speak("Already processed tasks! Ready to execute.",'error')
            else:
                self.publish_command(COMMANDS.START_EXECUTION)

        elif 'record' in command:
            if 'start' in command:
                self.publish_command(COMMANDS.START_RECORD)
            elif 'end' in command:
                self.publish_command(COMMANDS.END_RECORD)
            else:
                self.speak("No command found! Try again",'error')
                return
    
        elif ('process' in command):
            self.speak("No recording exists. Please record the taks first!",'error')
            
        elif ('execute' in command):
            self.speak("No tasks to execute. Please record the taks first!","error")

        elif('exit' in command):
            self.speak("Exiting...","warn")
        else:
            self.speak("No command found! Try again.","error")


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
    rclpy.init(args=args)

    voice = Voice()

    rclpy.spin(voice)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
