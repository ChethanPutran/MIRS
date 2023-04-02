#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import cv2
from dynamics.dynamics import Dynamics
from kinematics.kinematics import Kinematics
from trajectory.planning.trajectory_planner import TrajectoryPlanner
from trajectory.control.robot_state_publisher import RobotStatePublisher
from trajectory.control.trajectory_follower import TrajectoryFollower
from common.config import MODES,INPUT_EVENTS,JOINT_NAMES,JOINT_SENSOR_NAMES,JOINTS,JOINT_TYPE
from devices.motor import Motor
from devices.camera import Camera
from devices.sensors import JointSensor
from mirs_interfaces.msg import JointState
from trajectory.control.controllers import JointController

class Joint:
    def __init__(self,joint_name,Kp=1,Kd=1,Ki=1,joint_type=JOINT_TYPE['REVOLUTE']) -> None:
        self.motor = Motor('MOTOR_'+joint_name,JOINTS[joint_name])
        self.controller = JointController(joint_name,self.motor,Kp,Kd,Ki)
        self.type = joint_type
        self.name = joint_name
        
    def get_name(self):
        return self.name

class EndEffector:
    def __init__(self,joint_velocity=2):
        """
        joint_velocity in mm/s
        """
        self.JOINTS  = []
        # Initilaize joint motors
        for joint_name in JOINT_NAMES[7:]:
            self.JOINTS.append(Joint(joint_name))
        
        for joint in self.JOINTS:
            joint.set_velocity(joint_velocity)

class Robot(Node):   
    def __init__(self,event_handler,time_step=0.5):
        super().__init__("MIRS")
        self.MODES = MODES
        self.INPUT_EVENTS = INPUT_EVENTS
        self.TIME_STEP = 0.1  # in sec
        self.EXECUTION_ERROR = None
        self.EXECUTION_STATUS = None
        self.TIME_STEP = time_step  # in s
        self.SLEEP_TIME = 0.1  # in s
        self.STATE = {
            "Position": [0,0,0],
            "Velocity":[0,0,0,0,0,0],
        }
        self.LINKS = []
        self.kinematics = Kinematics()
        self.dynamics = Dynamics()
        self.trajectory_planner = TrajectoryPlanner()
        self.trajectory_follower = TrajectoryFollower()
        self.DEVICES = {}
        self.JOINTS = []
        self.JOINT_SENSORS = []
        self.event_handler = event_handler

        # Add end-effector
        self.EE = EndEffector()

        # Add camera
        self.CAMERA = Camera()
        
        # Initilaize joint motors
        for joint_name in JOINT_NAMES[:7]:
            self.JOINTS.append(Joint(joint_name))

        # Initilaize joint sensors
        for sensor_name in JOINT_SENSOR_NAMES:
            self.JOINT_SENSORS.append(JointSensor(name=sensor_name))

        self.create_subscription("/joint_state",JointState,self.update_state)
        self.robot_state_publisher = RobotStatePublisher()
    
    def update_state(self,state):
        self.matrices.update(state.data.theta1,
                             state.data.theta2,
                             state.data.theta3,
                             state.data.theta4,
                             state.data.theta5,
                             state.data.theta6,
                             )

    def set_velocity(self,velocity= 20 ):
        """
        velocity in mm/s
        """
        # Set velocity
        for joint in self.JOINTS:
            joint.set_velocity(velocity)

    def shut_down(self):
        self.event_handler.publish("shurdown")
    
    def execute(self,task_queue):
        Robot.EXECUTION_STATUS = True
        Robot.EXECUTION_ERROR = None
        
        while (len(task_queue) > 0 ):
            cur_task = task_queue.pop()
            sucess, error = self.perform(cur_task)
            if not sucess:
                print(error)
                if not sucess:
                    Robot.EXECUTION_STATUS = False
                    Robot.EXECUTION_ERROR = error
                    return Robot.EXECUTION_STATUS, Robot.EXECUTION_ERROR
            else:
                print('.', end='')
                #time.sleep(Robot.SLEEP_TIME)

        return Robot.EXECUTION_STATUS, Robot.EXECUTION_ERROR
    
    def perform(self,task):
        start_point = task.start_position
        end_point = task.end_position
        task_object = task.object
        action = task.action
        initial_speed = task.initial_speed 
        final_speed = task.final_speed 
        initial_acceleration = task.initial_acceleration 
        final_acceleration = task.final_acceleration 

        # Compute inverse kinematics

        # Starting point
        theta1 = self.kinematics.inverse(*start_point)
        theta1_dot = self.kinematics.compute_joint_velocity(initial_speed)

        # Ending point
        theta2 = self.kinematics.inverse(*end_point)
        theta2_dot = self.kinematics.compute_joint_velocity(final_speed)

        # Compute trajectory
        trajectory = self.trajectory_planner.plan(theta1,
                                                  theta2,
                                                  theta1_dot,
                                                  theta2_dot,
                                                  initial_acceleration,
                                                  final_acceleration
                                                  )
        # Follow trajectory and perform action
        status, error = self.trajectory_follower.follow_and_act(self,trajectory,action,task_object)

        return status, error

    def get_time(self):
        return self.time

    def get_state(self):
        return self.STATE
    
    def set_state(self):
        self.STATE['Position'] = [joint_sensor.get_position() for joint_sensor in self.JOINT_SENSORS]
        self.STATE['Velocity'] = [joint_sensor.get_velocity() for joint_sensor in self.JOINT_SENSORS]
    
   
    