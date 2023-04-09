#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mirs_interfaces.topics.topics import TOPICS
from mirs_interfaces.msg import TaskRecorderState
from .ai.task.recorder import Recorder

class RecorderState:
    INACTIVE = 'inactive'
    RECORDED = 'recorded'
    RECORDING = 'recording'
    FAILURE = 'failure'

class TaskRecorder(Node):
    def __init__(self,time_period=1):
        super().__init__("TaskRecorder")
        self.TIME_PERIOD = time_period
        self.state = {
            'status':'',
            'recording':None,
            'error':None
        }
        self.recorder = Recorder(fps=3)
        self.cmd = None
        # Create publisher
        self.recording_state_publisher = self.create_publisher(TaskRecorderState,TOPICS.TOPIC_RECORDER_STATUS,1)
        self.timer = self.create_timer(self.TIME_PERIOD,self.publish_recorder_state)

        # Create listener
        self.create_subscription(String,TOPICS.TOPIC_RECORDING,self.set_recording_cmd)

        self.set_state(RecorderState.INACTIVE)

    def publish_recorder_state(self):
        self.get_logger().info('Recorder status :'+self.state['status'])

        msg = TaskRecorderState()

        msg.status = self.state['status']
        msg.tasks = self.state['recording']
        msg.error = self.state['error']

        self.recording_state_publisher.publish(msg)
        self.exec_cmd()
        
    def set_state(self,status,recording=None,error=None):
        self.state['status'] = status
        self.state['recording'] = recording
        self.state['error'] = error

    def exec_cmd(self):
        self.get_logger().info("Exec cmd")
        if (self.get_recording_cmd() == "START_RECORD") and not(self.state['status'] == RecorderState.RECORDING):
            #Start record 
            self.set_state(RecorderState.RECORDING)
            self.recorder.record()
        elif self.get_recording_cmd() == "STOP_RECORD" and self.state['status'] == RecorderState.RECORDING:
            self.recorder.stop_record()
            self.set_state(RecorderState.RECORDED,self.recorder.get_recording())
    
    def set_recording_cmd(self,msg):
        self.cmd = msg.data

    def get_recording_cmd(self):
        return self.cmd

    
def main(args=None):
    rclpy.init(args=args)

    # Create node
    recorder_node = Recorder()

    # Run continuosly
    rclpy.spin(recorder_node)

    # Destroy node
    rclpy.shutdown()


if __name__ =="__main__":
    try:
        main()
    except Exception as e:
        print(str(e))