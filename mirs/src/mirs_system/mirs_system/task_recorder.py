#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mirs_system.conf.topics import TOPICS
from mirs_interfaces.msg import TaskRecorderState, SystemState
from mirs_system.ai.vision.camera import camera_client
from .conf.states import States
from .conf.commands import COMMANDS
import time


class TaskRecorder(Node):
    def __init__(self, time_period=5):
        super().__init__("TaskRecorder")
        self.TIME_PERIOD = time_period
        self.state = {
            'status': '',
            'message': '',
            'recording': [],
            'error': None
        }
        self.connected = False

        # Create recorder state publisher
        self.recording_state_publisher = self.create_publisher(
            TaskRecorderState, TOPICS.TOPIC_RECORDER_STATUS, 1)

        # self.timer = self.create_timer(
        #     self.TIME_PERIOD, self.publish_recorder_state)
        
        # Create system state listener
        self.create_subscription(
            SystemState, TOPICS.TOPIC_SYSTEM_STATE, self.set_recording_cmd, 1)

        self.set_state(States.INACTIVE)
        
        self.connect()
        

    def connect(self):
        try:
            self.camera_client = camera_client.CameraClient()
            self.connected = True
            self.set_state(States.ACTIVE, message="Connected to camera!")
        except Exception as e:
            self.set_state(States.ERROR, error="Could not connect to camera! Camera is offline. Trying to reconnect in 3 seconds.")
            time.sleep(3)
            self.connect()


    def publish_recorder_state(self):
        log = f" Recorder status: {self.state['status']} :{self.state['error'] if self.state['status']==States.ERROR else self.state['message']}"
        self.get_logger().info(log)

        msg = TaskRecorderState()

        msg.status = self.state['status']
        msg.error = self.state['error']
        msg.mssg = self.state['message']
        msg.recording = self.state['recording']
        msg.command_id = self.state['command_id']

        self.recording_state_publisher.publish(msg)
        self.clear_data()
    
    def clear_data(self):
        self.state['error'] = ''
        self.state['message'] = ''

    def set_state(self, status='', message='', recording=[], error='',command_id=''):
        if status:
            self.state['status'] = status
        self.state['recording'] = recording
        self.state['error'] = error
        self.state['message'] = message
        self.state['command_id'] = command_id

        self.publish_recorder_state()

    def get_recording_status(self):
        return self.state['status']

    def set_recording_cmd(self, msg:SystemState):
        if not msg.command:
            return

        # Recieved recording
        if msg.status == States.DONE:
            self.set_state(States.INACTIVE,command_id=msg.command_id)

        self.get_logger().info("Got command :"+msg.command)
        if 'record' not in msg.command:
            return

        recorder_status = self.get_recording_status()

        if (msg.command == COMMANDS.START_RECORD):
            if (recorder_status == States.RECORDING):
                self.set_state(command_id=msg.command_id,error="Recorder is alredy recording!")
                return

            res = self.camera_client.start_recording()

            if res['error']:
                self.get_logger().info((res["error"]))
                self.set_state(States.ERROR,command_id=msg.command_id, error=res['error'])
            else:
                self.get_logger().info(res["message"])
                self.set_state(States.RECORDING,command_id=msg.command_id, message=res['message'])

        elif msg.command == COMMANDS.END_RECORD:
            print(recorder_status)
            if not (recorder_status == States.RECORDING):
                self.set_state(command_id=msg.command_id,error="Recorder is not recording!")
                return

            res = self.camera_client.end_recording()

            if res['error']:
                self.get_logger().info("ERROR : "+res["error"])
                self.set_state(States.ERROR,command_id=msg.command_id, error=res['error'])
            else:
                self.set_state(States.RECIEVE_RECORDING,command_id=msg.command_id, message=res["message"]+" Recieving recording from server. It may take few minutes!")
                self.get_logger().info("Getting recording from server...")
                recording, error = self.camera_client.get_recording()

                if error:
                    self.get_logger().info(error)
                    self.set_state(error=error,command_id=msg.command_id,)
                else:
                    self.get_logger().info("Got recording.") 
                    self.get_logger().info(res["message"])
                    self.set_state(States.RECORDED,command_id=msg.command_id, message="Recording recieved. Waiting for the command to process.",recording=recording)
        elif msg.command == COMMANDS.EXIT:
            self.destroy_node()
            rclpy.shutdown()
            exit(0)

            
def main(args=None):
    recorder_node = None
    try:
        rclpy.init(args=args)

        # Create node
        recorder_node = TaskRecorder()

        # Run continuosly
        rclpy.spin(recorder_node)
    except Exception as e:
        print(e)
    finally:
        #Close camera client
        if recorder_node:
            recorder_node.camera_client.close()

        # Destroy node
        rclpy.shutdown()


if __name__ == "__main__":
    main()
