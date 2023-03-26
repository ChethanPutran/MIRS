#! usr/bin/env python3

from mirs_system.task_recorder import Recorder
from mirs_system.voice import Voice
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class System(Node):
    def __init__(self):
        super.__init__("system")
        self.recorder = Recorder()
        self.voice = Voice()
        self.timer = self.create_timer(1,self.run)
        self.commander = self.create_publisher(String,"/topic_sys_command",10)

    def run(self):
        cmd = self.voice.listen()
        self.get_logger().info(cmd)
        msg = String()
        msg.data = cmd


        if(cmd=="record" and not(self.recorder.state == "recording")):
            self.recorder.record()
        

def main(args=None):
    rclpy.init(args=args)

    system = System()

    rclpy.spin(system)

    system.destroy_node()

    rclpy.shutdown()


if __name__ =="__main__":
    main()