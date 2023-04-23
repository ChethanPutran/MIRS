# Importing neccessary modules
import datetime
import speech_recognition as sr
import pyttsx3 as ts
import os
from  mirs_interfaces.msg import VoiceState,SystemState
from  mirs_system.conf.topics import TOPICS
import rclpy
from rclpy.node import Node
import time

# Node
class Voice(Node):
    def __init__(self):
        super().__init__("MIRS_Voice")
        self.engine = ts.init()
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = 1
        self.START = True
        self.listening_period = 5 #sec

        # Rate
        self.engine.setProperty('rate', 125)

        # Volume 
        self.volume = self.engine.getProperty('volume')
        self.get_logger().info("Volume : "+str(self.volume))
        self.engine.setProperty('volume',1.0)

    
        # Initializing voice for Bot
        self.voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', self.voices[2].id)
        self.pause = None
        
        self.wish()
        
        self.create_subscription(SystemState,TOPICS.TOPIC_SYSTEM_STATE,self.callback,1)
        self.timer = self.create_timer(self.listening_period,self.listen_cmd)
        self.voice_cmd_publisher = self.create_publisher(VoiceState,TOPICS.TOPIC_VOICE_STATE,1)



    def listen_cmd(self):
        self.get_logger().info("Listening...")
        cmd = self.listen()
        self.publish_cmd(cmd)

    def publish_cmd(self,cmd):
        msg = VoiceState()
        msg.cmd = cmd 
        self.voice_cmd_publisher.publish(msg)

    def callback(self,msg):
        self.speak(message=msg.msg,tune=msg.type)

    def speak(self,audio,tune='normal'):
        if tune == "error":
            self.engine.say("Error!")
        elif tune == "warning":
            self.engine.say("Warning!")
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

    def listen(self):
        # It takes microphone input from the user and returns string output
        self.speak('Listening sir')
        with sr.Microphone() as source:
            print("Listening...")
            audio = self.recognizer.listen(source, timeout=5,phrase_time_limit=5)
            print("Recognizing...")

            # DIR_NAME = os.path.dirname(os.path.realpath(__file__))
            # TENSOR_GRAPH = os.path.join(DIR_NAME, "model\\yolov3.cfg")
            # TENSOR_LABELS = os.path.join(DIR_NAME, "model\\yolov3.cfg")
            # command = self.recognizer.recognize_tensorflow(
            #     audio, tensor_graph=TENSOR_GRAPH, tensor_label=TENSOR_LABELS)
            command = self.recognizer.recognize_google(audio)
        return command


def main(args=None):
    rclpy.init(args=args)

    voice = Voice()

    rclpy.spin(voice)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
