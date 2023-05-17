# Importing neccessary modules
import datetime
import pyttsx3 as ts
from  mirs_interfaces.msg import SystemState
from  mirs_system.conf.topics import TOPICS
import rclpy
from rclpy.node import Node

# Node
class VoiceOutput(Node):
    def __init__(self):
        super().__init__("MIRS_Voice_output")
        self.timer_period = 5
        self.message_queue = []
        self.engine = ts.init()
        self.volume = self.engine.getProperty('volume')
        self.engine.setProperty('volume',1.0)

        # Initializing voice for Bot
        self.voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', self.voices[0].id)
        
        self.wish()

        self.create_subscription(SystemState,TOPICS.TOPIC_SYSTEM_STATE,self.sys_state_callback,5)
        self.timer = self.create_timer(self.timer_period,self.handle_messages)

    def handle_messages(self):
        if len(self.message_queue)>0:
            (msg,tune) = self.message_queue.pop(0)
            self.get_logger().info('Got  :'+tune+". Data : "+msg)
            self.speak(msg,tune=tune)

    def sys_state_callback(self,msg:SystemState):
        if (not msg.error) and (not  msg.mssg):
            return
        
        self.get_logger().info("Got sys state :"+msg.error+msg.mssg)
        
        data = msg.mssg
        tune = 'normal'

        if msg.error:
            tune = 'error'
            data = msg.error
        
        self.message_queue.append((data,tune))

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



def main(args=None):
    rclpy.init(args=args)

    voice = VoiceOutput()

    rclpy.spin(voice)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
