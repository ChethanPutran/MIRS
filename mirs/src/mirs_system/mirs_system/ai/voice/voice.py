# Importing neccessary modules
import datetime
import speech_recognition as sr
import pyttsx3 as ts
import os
from  mirs_interfaces.msg import VoiceState,SytemState
from  mirs_interfaces.topics.topics import TOPICS
import rclpy
from rclpy.node import Node

# Node
class Voice(Node):
    def __init__(self, voice_male=True):
        self.engine = ts.init('sapi5')
        self.recognizer = sr.Recognizer()
        self.START = True
        self.listening_period = 5 #sec
        self.create_subscription(TOPICS.TOPIC_SYSTEM_STATE,SytemState,self.callback)
        self.timer = self.create_timer(self.listening_period,self.listen_cmd)
        self.voice_cmd_publisher = self.create_publisher(TOPICS.TOPIC_VOICE_STATE,VoiceState,1)

        # Initializing voice for Bot
        self.voices = self.engine.getProperty('voices')

        # print(voices[0])
        if (voice_male):
            self.engine.setProperty('voice', self.voices[0].id)
        else:
            self.engine.setProperty('voice', self.voices[1].id)
        self.pause = None

        self.wish()

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
            audio = self.recognizer.listen(source, 10)
            print("Recognizing...")

            DIR_NAME = os.path.dirname(os.path.realpath(__file__))
            TENSOR_GRAPH = os.path.join(DIR_NAME, "model\\yolov3.cfg")
            TENSOR_LABELS = os.path.join(DIR_NAME, "model\\yolov3.cfg")
            # command = self.recognizer.recognize_tensorflow(
            #     audio, tensor_graph=TENSOR_GRAPH, tensor_label=TENSOR_LABELS)
            command = self.recognizer.recognize_google(audio)
        return command


def main(args=None):
    rclpy.init(args)

    system = Voice()

    rclpy.spin(system)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
