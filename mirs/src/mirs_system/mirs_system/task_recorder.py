#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mirs_system.conf.topics import TOPICS
from mirs_interfaces.msg import TaskRecorderState, SystemState
from mirs_system.ai.vision.camera import camera_client
from .conf.states import States
from .conf.commands import COMMANDS


class TaskRecorder(Node):
    def __init__(self, time_period=3):
        super().__init__("TaskRecorder")
        self.TIME_PERIOD = time_period
        self.state = {
            'status': '',
            'message': '',
            'recording': None,
            'error': None
        }

        # Create recorder state publisher
        self.recording_state_publisher = self.create_publisher(
            TaskRecorderState, TOPICS.TOPIC_RECORDER_STATUS, 1)

        self.timer = self.create_timer(
            self.TIME_PERIOD, self.publish_recorder_state)

        # Create system state listener
        self.create_subscription(
            SystemState, TOPICS.TOPIC_SYSTEM_STATE, self.set_recording_cmd, 1)

        self.set_state(States.INACTIVE)

        try:
            self.camera_client = camera_client.CameraClient("localhost")
        except Exception as e:
            self.set_state(States.ERROR, error=str(e))

    def publish_recorder_state(self):
        log = f" Recorder status: {self.state['status']} :{self.state['error'] if self.state['status']==States.ERROR else self.state['message']}"
        self.get_logger().info(log)

        msg = TaskRecorderState()

        msg.status = self.state['status']
        msg.error = self.state['error']
        msg.mssg = self.state['message']
        msg.recording = self.state['recording']

        self.recording_state_publisher.publish(msg)

    def set_state(self, status='', message='', recording='', error=''):
        if status:
            self.state['status'] = status
        self.state['recording'] = recording
        self.state['error'] = error
        self.state['message'] = message

    def get_recording_status(self):
        return self.state['status']

    def set_recording_cmd(self, msg:SystemState):

        # Recieved recording
        if msg.status == States.DONE:
            self.set_state(States.INACTIVE)

        self.get_logger().info("Got command :"+msg.command)
        if 'record' not in msg.command:
            return

        recorder_status = self.get_recording_status()

        if (msg.command == COMMANDS.START_RECORD):
            if (recorder_status == States.RECORDING):
                self.set_state(error="Recorder is alredy recording!")
                return

            res = self.camera_client.start_recording()

            if res['error']:
                self.get_logger().info((res["error"]))
                self.set_state(States.ERROR, error=res['error'])
            else:
                self.get_logger().info(res["message"])
                self.set_state(States.RECORDING, message=res['message'])

        elif msg.command == COMMANDS.END_RECORD:
            print(recorder_status)
            if not (recorder_status == States.RECORDING):
                self.set_state(error="Recorder is not recording!")
                return

            res = self.camera_client.end_recording()

            if res['error']:
                self.get_logger().info((res["error"]))
                self.set_state(States.ERROR, error=res['error'])
            else:
                self.get_logger().info(res["message"])
                self.set_state(States.RECORDED, message=res['message'])

        elif msg.command == COMMANDS.GET_RECORDING:
            recording, error = self.camera_client.get_recording()
            print(" Get recording err", error)

            if error:
                self.get_logger().info(error)
                self.set_state(error=error)
            else:
                self.set_state(States.RECIEVE_RECORING, recording=recording)


def main(args=None):
    rclpy.init(args=args)

    # Create node
    recorder_node = TaskRecorder()

    # Run continuosly
    rclpy.spin(recorder_node)

    # Destroy node
    rclpy.shutdown()


if __name__ == "__main__":
    main()
