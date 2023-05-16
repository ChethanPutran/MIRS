import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

from mirs_interfaces.devices.camera_client import CameraClient
from mirs_interfaces.msg import Error,Task,RobotState
from mirs_interfaces.hardware.hardware import Hardware
from mirs_system.mirs_system.conf.topics import TOPICS
from mirs_system.mirs_system.conf.commands import COMMANDS

from .kinematics.kinematics import Kinematics
from .trajectory.planning.trajectory_planner import TrajectoryPlanner
from .trajectory.control.trajectory_follower import TrajectoryFollower  
from .common.config import MODES,INPUT_EVENTS,JOINT_NAMES,JOINT_TYPE
from .transformations.matrices import Matrices
from .trajectory.planning.end_effector_planner import EEPlanner

class Joint:
    def __init__(self,joint_name,Kp=1,Kd=1,Ki=1,joint_type=JOINT_TYPE['REVOLUTE']):
        self.name = joint_name
        self.type = joint_type
        self.Kp  = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.state = [0,0]
        
    def get_name(self):
        return self.name
    def get_state(self):
        return self.name
    def set_state(self,theta,theta_dot):
        self.state[0] = theta
        self.state[1] = theta_dot

class EndEffector:
    def __init__(self,joint_velocity=2):
        """
        joint_velocity in mm/s
        """
        self.JOINTS  = []
        # Initilaize joint motors
        for joint_name in JOINT_NAMES[7:]:
            self.JOINTS.append(Joint(joint_name))
        
        # for joint in self.JOINTS:
        #     joint.set_velocity(joint_velocity)
    

class Robot(Node):   
    def __init__(self,n=6,time_step=1):
        super().__init__("MIRS_ROBOT")
        self.get_logger().info("Creating mirs robot node...")
        self.n = n
        self.STATE = {
            "theta":[0]*self.n,
            "theta_dot":[0]*self.n,
            "theta_dotdot":[0]*self.n,
            "torque":[0]*self.n,
            "theta":[0]*self.n,
            "position": [0,0,0],
            "velocity": [0,0,0],
            "pose": [0,0,0]
        }
        self.TIME_STEP = time_step  # in s
        self.SLEEP_TIME = 0.1  # in s
        self.time = time.time()

         # self.kinematics = Kinematics()
        self.trajectory_planner = TrajectoryPlanner()
        self.trajectory_follower = TrajectoryFollower()
        self.ee_planner = EEPlanner() 
        # self.matrices = Matrices
        # self.EE = EndEffector()
        # self.CAMERA = CameraClient()

        self.JOINTS = []

        self.joint_state_publisher = self.create_publisher(JointState,TOPICS.TOPIC_JOINT_STATE,1)
        self.robot_state_publisher = self.create_publisher(RobotState,TOPICS.TOPIC_ROBOT_STATE,1)
        self.task_subcriber = self.create_subscription(Task,TOPICS.TOPIC_TASK,self.new_task_callback,1)

        self.timer = self.create_timer(self.TIME_STEP,self.update_state)

        
        for joint_name in JOINT_NAMES[:7]:
            self.JOINTS.append(Joint(joint_name))


        # self.MODES = MODES
        # self.INPUT_EVENTS = INPUT_EVENTS
        # self.EXECUTION_ERROR = None
        # self.EXECUTION_STATUS = None
    
    def new_task_callback(self,msg:Task):
        self.get_logger().info("New task obtained.")
        print(msg)

        res = RobotState()
        res.error = ''
        res.finished_task = msg.id
     
        # sucess, error = self.perform(msg)
        # if not sucess:
        #     print(error)
        #     if not sucess:
        #         res.finished_task = ''
        #         res.error = error
        # self.robot_state_publisher.publish(res)

    
    """ Get feedback from hardware and update robot state / pose """
    def update_state(self):
        self.get_logger().info("Updating robot state...")
    
    
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
        
        action_sequence = self.ee_planner.get_action_sequence(action,task_object,trajectory.time_length)

        sucessfull,error = self.trajectory_follower.follow_and_act(trajectory,action_sequence)

     
        if sucessfull:
            return True,None
        return False,error

        
    def get_time(self):
        return self.time

    def get_state(self):
        return self.STATE
    
    def set_state(self,msg):
        print(msg)
        # Hardware.set_state()
        # self.STATE['Pose'] = self.matrices.get_ee_pose()
        # self.STATE['Position'] = self.kinematics.forward(self.theta)
        # self.STATE['Velocity'] = self.kinematics.compute_ee_velocity(self.theta_dot)
    

def main(args=None):
    rclpy.init(args=args)
    
    robot = Robot()

    rclpy.spin(robot)

    robot.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()