
from tkinter import Tk, Button, END, Text
import os
from mirs_interfaces.msg import GuiState, SystemState
from mirs_system.conf.topics import TOPICS
import rclpy
from rclpy.node import Node
import threading


class GUI(Node):
    def __init__(self):
        super().__init__("MIRS_GUI")
        self.cmd = ''
        self.pre_cmd = ''

        self.window = Tk()

        self.window.wm_title("MIRS Voice")

        # Create text widget and specify size.
        self.display = Text(self.window, height=5, width=52, bg='white')
        self.display.grid(row=0, columnspan=6)

        b1 = Button(self.window, text="Record", width=12, command=self.record)
        b1.grid(row=1, column=2)

        b2 = Button(self.window, text="Execute",
                    width=12, command=self.execute)
        b2.grid(row=1, column=3)

        b3 = Button(self.window, text="Extract",
                    width=12, command=self.extract)
        b3.grid(row=1, column=4)

        b4 = Button(self.window, text="Exit", width=12, command=self.exit)
        b4.grid(row=1, column=5)

        self.create_subscription(
            SystemState, TOPICS.TOPIC_SYSTEM_STATE, self.print_cmd, 1)
        self.timer = self.create_timer(1, self.send_cmd)
        self.cmd_publisher = self.create_publisher(
            GuiState, TOPICS.TOPIC_GUI_STATE, 1)

    def send_cmd(self):
        print('.')
        print(self.cmd)
        if self.pre_cmd and not (self.cmd == self.pre_cmd):
            self.get_logger().info("Publishing command :"+self.cmd)
            msg = GuiState()
            msg.command = self.cmd
            self.cmd_publisher.publish(msg)
            self.pre_cmd = self.cmd

            if self.cmd == "exit":
                self.window.destroy()
                self.destroy_node()

    def print_cmd(self, message):
        self.display.delete("1.0", "end")
        self.display.insert(END, message)

    def set_cmd(self, cmd):
        self.cmd = cmd

    def record(self):
        self.set_cmd('record')
        self.print_cmd('Recording...')

    def extract(self):
        self.set_cmd('extract')
        self.print_cmd('Extracting...')

    def execute(self):
        self.set_cmd('execute')
        self.print_cmd('Executing...')

    def exit(self):
        self.set_cmd('exit')


def main(args=None):

    rclpy.init(args=args)

    gui = GUI()

    rclpy.spin(gui)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
