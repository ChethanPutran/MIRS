#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from .kinematics.kinematics import Kinematics
from .trajectory.planning.trajectory_planner import TrajectoryPlanner
from .common.config import MODES,INPUT_EVENTS,JOINT_NAMES,JOINT_TYPE
from .devices.camera import Camera
from mirs_interfaces.msg import JointState,Error,RobotState,Trajectory
from .transformations.matrices import Matrices
from .trajectory.planning.end_effector_planner import EEPlanner
from mirs_interfaces.topics.topics import TOPICS
from mirs_interfaces.topics.services import SERVICES
from mirs_interfaces.srv import ExecTask

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
    def __init__(self,time_step=0.5):
        super().__init__("MIRS_ROBOT")
        self.get_logger().info("Creating mirs robot node...")
        self.DOF = 6
        self.MODES = MODES
        self.INPUT_EVENTS = INPUT_EVENTS
        self.TIME_STEP = 0.1  # in sec
        self.EXECUTION_ERROR = None
        self.EXECUTION_STATUS = None
        self.TIME_STEP = time_step  # in s
        self.SLEEP_TIME = 0.1  # in s
        self.theta = np.zeros((1,6))
        self.theta_dot = np.zeros((1,6))
        self.time = time.time()
        self.STATE = {
            "Position": [0,0,0],
            "Velocity": [0,0,0],
            "Pose": []
        }
        self.execute_task_srv = self.create_service(ExecTask,SERVICES.SERVICE_EXECUTE_TASK,self.execute_tasks_callback)
        self.kinematics = Kinematics()
        self.trajectory_planner = TrajectoryPlanner()
        self.ee_planner = EEPlanner() 
        self.JOINTS = []
        self.create_subscription(JointState,TOPICS.TOPIC_JOINT_STATE,self.update_state,1)
        self.robot_state_publisher = self.create_publisher(RobotState,TOPICS.TOPIC_ROBOT_STATE,1)
        self.error_publisher = self.create_publisher(Error,TOPICS.TOPIC_ERROR,1)
        self.trajectory_publisher = self.create_publisher(Trajectory,TOPICS.TOPIC_TRAJECTORY,1)
        self.matrices = Matrices
        self.EE = EndEffector()
        self.CAMERA = Camera()
        
        for joint_name in JOINT_NAMES[:7]:
            self.JOINTS.append(Joint(joint_name))


    """ Subscription to joint states from Joint State Publisher"""
    def update_state(self,state):
        self.time = time.time()
        self.theta[:,:] = [state.theta]

        # Update pose
        self.matrices.update(*state.theta)
        self.theta_dot[:,:] = state.theta_dot

        # Update joint state
        for idx,joint in enumerate(self.JOINTS):
            joint.set_state(self.theta[idx],self.theta_dot[idx])


    def shut_down(self):
        self.event_handler.publish("shurdown")
    
    def execute_tasks_callback(self,req,res):
        task_queue = req.TASKS

        res.EXECUTION_STATUS = True
        res.EXECUTION_ERROR = None
        
        while (not task_queue.is_over()):
            cur_task = task_queue.pop()
            sucess, error = self.perform(cur_task)
            if not sucess:
                print(error)
                if not sucess:
                    res.EXECUTION_STATUS = False
                    res.EXECUTION_ERROR = error
                    return res
            else:
                print('.', end='')
                #time.sleep(Robot.SLEEP_TIME)

        return res
    
    def perform(self,task):
           for task in tasks:
            msg = Task()
            msg.object = task.object
            msg.action = task.action
            msg.theta = task.theta
            msg.theta_dot = task.theta_dot

            # Create service
            self.task_publisher.publish(msg)

            # Wait for the response
            sucess,msg = self.get_task_feedback()

            # continue with next task if sucess
            if sucess:
                task_exec.append((task.id,task.action))
                continue
            else:
                self.set_state(TaskExecutorStatus.FAILURE)
                return
        self.state = TaskExecutorStatus.EXECUTED
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
        
        self.pulish_trajectory()
        
        action_sequence = self.ee_planner.get_action_sequence(action,task_object,trajectory.time_length)

        msg = Trajectory()
        msg.action_seqence = action_sequence
        msg.trajectory = trajectory

        # Follow trajectory and perform action

        self.trajectory_publisher.publish(msg)

        # Wait for the response 
        # if sucessfull:
        #     return True
        # return False

    def get_time(self):
        return self.time

    def get_state(self):
        return self.STATE
    
    def set_state(self):
        self.STATE['Pose'] = self.matrices.get_ee_pose()
        self.STATE['Position'] = self.kinematics.forward(self.theta)
        self.STATE['Velocity'] = self.kinematics.compute_ee_velocity(self.theta_dot)
    
    def report_error(self,error):
        self.error_publisher.publish(error)

def main(args=None):
    rclpy.init(args=args)
    
    robot = Robot()

    rclpy.spin(robot)

    robot.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()