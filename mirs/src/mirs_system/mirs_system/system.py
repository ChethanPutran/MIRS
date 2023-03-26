#!/usr/bin/env python3
import rclpy
from rclpy.clock import Clock
import argparse
from mirs_controller.mirs_controller.robot import Robot
from mirs_controller.mirs_controller.joint_state_publisher import JointStatePublisher
from mirs_controller.mirs_controller.trajectory_follower import TrajectoryFollower


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node-name', dest='nodeName', default='ur_driver', help='Specifies the name of the node.')
    arguments, unknown = parser.parse_known_args()

    rclpy.init_node(arguments.nodeName, disable_signals=True)

    jointPrefix = rclpy.get_param('prefix', '')
    if jointPrefix:
        print('Setting prefix to %s' % jointPrefix)

    robot = Robot()
    nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''
    jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
    trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, nodeName)
    trajectoryFollower.start()

    # we want to use simulation time for ROS
    clockPublisher = rclpy.Publisher('clock', Clock, queue_size=1)
    if not rclpy.get_param('use_sim_time', False):
        rclpy.logwarn('use_sim_time is not set!')

    timestep = int(robot.getBasicTimeStep())

    while robot.step(timestep) != -1 and not rclpy.is_shutdown():
        jointStatePublisher.publish()
        trajectoryFollower.update()
        # pulish simulation clock
        msg = Clock()
        time = robot.getTime()
        msg.clock.secs = int(time)
        # round prevents precision issues that can cause problems with ROS timers
        msg.clock.nsecs = int(round(1000 * (time - msg.clock.secs)) * 1.0e+6)
        clockPublisher.publish(msg)
