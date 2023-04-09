#!/usr/bin/env python

from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy_tutorials.msg import Floats
import math

class ArduinoPublisher(Node):
    def __init__(self,name):
        super().__init__(f"ArduinoPublisher_{name}")
        self.publisher = self.create_publisher('arduino',String)

    def cb(self,msg):
        x=Floats()
        x.data.append(180+(math.degrees(msg.position[0])))
        x.data.append(180+(math.degrees(msg.position[1])))
        x.data.append(180+(math.degrees(msg.position[2])))
        
        x.data.append(180+(math.degrees(msg.position[3])))
        x.data.append(180+(math.degrees(msg.position[4])))
        x.data.append(180+(math.degrees(msg.position[5])))
        #rclpy.sleep(1/2)
        self.publisher.publish(x)
        
class ArduinoSuscriber(Node):
    def __init__(self):
        super().__init__("ArduinoSuscriber")
        self.publisher = self.create_subscription('arduino',String,self.cb)

    def cb(self,msg):
        x=Floats()
        x.data.append(180+(math.degrees(msg.position[0])))
        x.data.append(180+(math.degrees(msg.position[1])))
        x.data.append(180+(math.degrees(msg.position[2])))
        
        x.data.append(180+(math.degrees(msg.position[3])))
        x.data.append(180+(math.degrees(msg.position[4])))
        x.data.append(180+(math.degrees(msg.position[5])))
        #rclpy.sleep(1/2)
    


