import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterValue
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
# from tf2_ros import TransformBroadcaster, TransformStamped

# from mirs_interfaces.devices.camera_client import CameraClient
# from mirs_interfaces.msg import Error,Task,RobotState
# from mirs_interfaces.hardware.hardware import Hardware
from mirs_system.conf.topics import TOPICS
# from mirs_system.conf.commands import COMMANDS

from .kinematics.kinematics import Kinematics
# from .trajectory.planning.trajectory_planner import TrajectoryPlanner
# from .trajectory.control.trajectory_follower import TrajectoryFollower  
# from .common.config import MODES,INPUT_EVENTS,JOINT_NAMES,JOINT_TYPE
from .common.urdf_converter import URDFConverter
# from .transformations.matrices import Matrices
# from .trajectory.planning.end_effector_planner import EEPlanner
from .hardware.controller import Controller
# from .hardware.console import Console
from .kinematics.kinematics import Kinematics
from .trajectory.trajectory_generator import Trajectory,TrajectoryGenerator
from .dynamics.dynamics import Dynamics
from .transformations.transform import Transform

# from ai import AI


# class Joint:
#     def __init__(self,joint_name,Kp=1,Kd=1,Ki=1,joint_type=JOINT_TYPE['REVOLUTE']):
#         self.name = joint_name
#         self.type = joint_type
#         self.Kp  = Kp
#         self.Kd = Kd
#         self.Ki = Ki
#         self.state = [0,0]
        
#     def get_name(self):
#         return self.name
    
#     def get_state(self):
#         return self.name
    
#     def set_state(self,theta,theta_dot):
#         self.state[0] = theta
#         self.state[1] = theta_dot

# class EndEffector:
#     def __init__(self,joint_velocity=2):
#         """
#         joint_velocity in mm/s
#         """
#         self.JOINTS  = []
#         # Initilaize joint motors
#         for joint_name in JOINT_NAMES[7:]:
#             self.JOINTS.append(Joint(joint_name))
        
#         # for joint in self.JOINTS:
#         #     joint.set_velocity(joint_velocity)
    
class Robot(Node):   
    # COLORS = ['red', 'orange', 'crimson', 'magenta', 'blue', 'limegreen']
    # axis_colors = ['red', 'green', 'blue']
    # movements = {
    #     "MOVE_UP": np.array([[0], [0], [1]]),
    #     "MOVE_DOWN": np.array([[0], [0], [-1]]),
    #     "MOVE_LEFT": np.array([[0], [1], [0]]),
    #     "MOVE_RIGHT": np.array([[0], [-1], [0]]),
    #     "MOVE_FRONT": np.array([[1], [0], [0]]),
    #     "MOVE_BACK": np.array([[-1], [0], [0]]),
    # }
    
    
    def __init__(self):
        super().__init__("MIRS_ROBOT")

        # Robot parameters (supplied from launch files/CLI)
        # self.declare_parameter('robot_description', rclpy.Parameter.Type.STRING) 
        # # self.declare_parameter('debug_mode', rclpy.Parameter.Type.BOOL,ParameterValue(False).bool_value) 
        # self.declare_parameter('delta_t', 0.1) 
        # self.declare_parameter('precision',4) 
        # self.declare_parameter('simp_sim', rclpy.Parameter.Type.BOOL) 

        # self.delta_t = self.get_parameter('delta_t').value
        # # self.simp_sim = self.get_parameter('simp_sim')
        # self.model_file_name = self.get_parameter('robot_description').value
        # # self.debug = self.get_parameter('debug_mode')
        # self.precision = self.get_parameter('precision').value
        self.TIME_STEP =1

        # Robot model specs
        self.robot_model = URDFConverter(TEST_MODEL=True).get_robot_model()
        # self.robot_model = URDFConverter(self.model_file_name).get_robot_model()
        # self.n = self.robot_model.n
        self.n = 6
        # self.console  = Console(self.n)
        # self.console.loop()
        # self.console.set_robot_params(self.robot_model)


        # Robot state variables
        self.theta = np.zeros((self.n, 1))
        self.theta_d = np.zeros((self.n, 1))
        self.theta_dd = np.zeros((self.n, 1))

        # Robot control variables
        self.Tau = np.zeros((self.n, 1))
        self.Tau_ee = np.zeros((3, 1))
        self.F_ee = np.zeros((3, 1))

        # Robot Software Components
        # self.controller = Controller(self)
        # self.transform = Transform(self.robot_model)
        # self.kinematics = Kinematics(self.transform, self.precision)
        # self.dynamics = Dynamics(self.transform, self.precision)
        # self.trajec_gen = TrajectoryGenerator(self.kinematics,
        #                                          self.dynamics,
        #                                          self.delta_t,
        #                                          precision=self.precision,
        #                                          trajectory_type=Trajectory.TRAJECTORY_CUBIC)
        
        # Node specific variable
        self.joint_state_publisher = self.create_publisher(JointState,TOPICS.TOPIC_JOINT_STATE,1)
        self.timer = self.create_timer(self.TIME_STEP,self.publish_state)
        # self.task_subcriber = self.create_subscription(Task,TOPICS.TOPIC_TASK,self.new_task_callback,1)

        # Simulation specific
        # self.colors = self.COLORS[:self.n+1]
        # self.links_in_frame = np.array(self.robot_model.links)
        # self.links = self.robot_model.links
        # self.time_stamp = None
        # self.u = 0
        # self.on_exit_callback = None

        # self.ai = AI()

        # self.init_home_pos()
        # self.get_logger().info("Creating mirs robot node...")
        # self.n = n
        # self.STATE = {
        #     "theta":[0]*self.n,
        #     "theta_dot":[0]*self.n,
        #     "theta_dotdot":[0]*self.n,
        #     "torque":[0]*self.n,
        #     "theta":[0]*self.n,
        #     "position": [0,0,0],
        #     "velocity": [0,0,0],
        #     "pose": [0,0,0]
        # }
        # self.TIME_STEP = time_step  # in s
        # self.SLEEP_TIME = 0.1  # in s
        # self.time = time.time()

        #  # self.kinematics = Kinematics()
        # self.trajectory_planner = TrajectoryPlanner()
        # self.trajectory_follower = TrajectoryFollower()
        # self.ee_planner = EEPlanner() 
        # # self.matrices = Matrices
        # # self.EE = EndEffector()
        # # self.CAMERA = CameraClient()

        # self.JOINTS = []

        # self.robot_state_subcriber = self.create_subscription(Task,TOPICS.TOPIC_TASK,self.new_task_callback,1)
        # self.robot_state_publisher = self.create_publisher(RobotState,TOPICS.TOPIC_ROBOT_STATE,1)
        # self.task_subcriber = self.create_subscription(Task,TOPICS.TOPIC_TASK,self.new_task_callback,1)

        # for joint_name in JOINT_NAMES[:7]:
        #     self.JOINTS.append(Joint(joint_name))

        # self.MODES = MODES
        # self.INPUT_EVENTS = INPUT_EVENTS
        # self.EXECUTION_ERROR = None
        # self.EXECUTION_STATUS = None
    
    # def new_task_callback(self,msg:Task):
    #     self.get_logger().info("New task obtained.")
    #     print(msg)

    #     res = RobotState()
    #     res.error = ''
    #     res.finished_task = msg.id
     
        # sucess, error = self.perform(msg)
        # if not sucess:
        #     print(error)
        #     if not sucess:
        #         res.finished_task = ''
        #         res.error = error
        # self.robot_state_publisher.publish(res)

    """ Get feedback from hardware and publish robot state / pose """
    def publish_state(self):
        self.get_logger().info("Updating robot state...")
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.robot_model.joint_names[:]
        msg.position = self.theta.flatten().tolist()
        msg.velocity = self.theta_d.flatten().tolist()
        msg.effort = self.Tau.flatten().tolist()

        self.joint_state_publisher.publish(msg)
        self.theta += 0.1
    
    # def perform(self,task):
    #     start_point = task.start_position
    #     end_point = task.end_position
    #     task_object = task.object
    #     action = task.action
    #     initial_speed = task.initial_speed 
    #     final_speed = task.final_speed 
    #     initial_acceleration = task.initial_acceleration 
    #     final_acceleration = task.final_acceleration 

    #     # Compute inverse kinematics

    #     # Starting point
    #     theta1 = self.kinematics.inverse(*start_point)
    #     theta1_dot = self.kinematics.compute_joint_velocity(initial_speed)

    #     # Ending point
    #     theta2 = self.kinematics.inverse(*end_point)
    #     theta2_dot = self.kinematics.compute_joint_velocity(final_speed)

    #     # Compute trajectory
    #     trajectory = self.trajectory_planner.plan(theta1,
    #                                             theta2,
    #                                             theta1_dot,
    #                                             theta2_dot,
    #                                             initial_acceleration,
    #                                             final_acceleration
    #                                             )
        
    #     action_sequence = self.ee_planner.get_action_sequence(action,task_object,trajectory.time_length)

    #     sucessfull,error = self.trajectory_follower.follow_and_act(trajectory,action_sequence)

     
    #     if sucessfull:
    #         return True,None
    #     return False,error
  
    # def get_time(self):
    #     return self.time

    # def get_state(self):
    #     return self.STATE
    
    # def set_state(self,msg):
    #     print(msg)
    #     # Hardware.set_state()
    #     # self.STATE['Pose'] = self.matrices.get_ee_pose()
    #     # self.STATE['Position'] = self.kinematics.forward(self.theta)
    #     # self.STATE['Velocity'] = self.kinematics.compute_ee_velocity(self.theta_dot)

    # def on_exit(self, callback):
    #     self.on_exit_callback = callback

    # def get_disp_vector(self, action, displacement):
    #     return self.movements[action]*displacement

    # def on_joint_move(self, callback):
    #     self.on_joint_move_callback = callback

    # def on_simp_sim(self, callback):
    #     self.on_simp_sim_callback = callback

    # def on_new_trajectory(self, callback):
    #     self.on_new_trajectory_callback = callback

    # def init_home_pos(self):
    #     # Compute home position
    #     for i in range(0, self.n):
    #         # L - 2x4 -> 4x2
    #         self.links[i] = np.dot(self.transform._0T(i), np.dot(
    #             self.transform.Rot(self.theta[i, 0]), self.links_in_frame[i].T))

    # def exit(self):
    #     print("Exiting from the program...")
    #     self.on_exit_callback()
    #     exit(0)

    # def move_to_goal(self, goal_point, T, simple_sim=False):
    #     start_orientation, start_point = self.transform.get_current_pose()

    #     ons = np.ones((self.n, 1))

    #     status, trajectory = self.trajec_gen.generate_trajectory(
    #         start_point, goal_point, 0, T, 0.1*ons, 0.2*ons, 0.3*ons)

    #     if (not status):
    #         # print(trajectory)
    #         return

    #     self.on_new_trajectory_callback(trajectory)

    #     # self.trajec_plotter.plot_trajectory(trajectory)
    #     trajectory.init_goal()

    #     if self.simp_sim:
    #         self.on_simp_sim_callback(trajectory, self.move_joint)
    #     else:
    #         return self.controller.execute(trajectory)

    # def add_noise(self, th, noise_mean=0, noise_var=np.pi/180):
    #     return np.round(th + np.random.normal(noise_mean, noise_var, size=(self.n, 1)), self.precision)

    # def step(self, u, time_stamp=None, add_noise=False):
    #     self.u = u
    #     self.time_stamp = time_stamp
    #     self.theta_dd[:, :] = self.dynamics.inverse(
    #         u, self.theta, self.theta_d)

    #     theta_d = self.theta_d + self.theta_dd * self.delta_t
    #     theta = self.theta + self.theta_d * \
    #         self.delta_t + self.theta_dd * self.delta_t**2

    #     if add_noise:
    #         theta = self.add_noise(theta)
    #         theta_d = self.add_noise(theta_d)

    #     if self.debug:
    #         print(f"T : {u}")
    #         print(f"theta :{theta}")
    #         print(f"theta_d :{theta_d}")
    #         print(f"theta_dd :{self.theta_dd}")

    #     self.move_joint(theta, theta_d, self.theta_dd)

    # def execute_program(self, file_name="program.txt"):
    #     actions = ["PICK", "PLACE"]
    #     movements = ["MOVE_DOWN", "MOVE_UP", "MOVE_LEFT",
    #                  "MOVE_RIGHT", "MOVE_FRONT", "MOVE_BACK", "MOVE"]

    #     # print("File path :", file_name)
    #     command_list = []
    #     args_list = []

    #     PROGRAM = """
    #     MOVE_UP 10
    #     MOVE_LEFT 30
    #     MOVE_BACK 20
    #     """

    #     with open(file_name,'w') as f:
    #         f.write(PROGRAM)

    #     with open(file_name, 'r') as f:
    #         for line in f.readlines():
    #             res = line.split()
    #             command_list.append(res[0])
    #             args_list.append(res[1:])

    #     for i in range(len(command_list)):
    #         print(f"Command :{command_list[i]} Args :{args_list[i]}")
    #         command = command_list[i]
    #         args = args_list[i]

    #         if (command in actions):
    #             # position = self.ai.get_object_position(args[0])
    #             position = np.array(args[1:4], dtype=np.float16).reshape(-1, 1)
    #             print(f"Action :{str.title(command)}ing object...")
    #             self.move_to_goal(position, int(args[-1]))
    #         else:
    #             print(f"Movement :{str.title(command)}")
    #             displacement_vec = self.get_disp_vector(
    #                 command, float(args[0]))
    #             print("Disp vec :", displacement_vec)
    #             print("Cur vec :", self.get_ee_position())
    #             if(self.move_to_goal(self.get_ee_position() +
    #                               displacement_vec, float(args[1]))):
    #                 continue
    #             else:
    #                 print("ERROR!!!")
    #                 break
    #         time.sleep(0.1)

    # def move_joint(self, theta, theta_d=np.ones((3, 1)), theta_dd=np.ones((3, 1))):
    #     if self.debug:
    #         print("Moving Theta : ", theta, "\n\n")

    #     self.theta[:, :] = theta
    #     self.theta_d[:, :] = theta_d
    #     self.theta_dd[:, :] = theta_dd

    #     self.transform.update(theta)
    #     self.update_link_pos()

    #     # Handled by the joint state topic publisher 
    #     # if (self.controller.controller_running):
    #     #     self.on_joint_move_callback(self.time_stamp,
    #     #                                 [self.controller.theta_ref,
    #     #                                  self.theta],
    #     #                                 [self.controller.theta_d_ref,
    #     #                                  self.theta_d],
    #     #                                 self.u)
    #     # else:
    #     #     self.on_joint_move_callback(self.time_stamp)

    # def update_link_pos(self):
    #     for i in range(0, self.n):
    #         self.links[i] = np.dot(self.transform._0T(i), np.dot(self.transform.Rot(
    #             self.theta[i, 0]), self.links_in_frame[i].T))

    # def get_joint_values(self):
    #     return self.theta, self.theta_d

    # def get_ee_static_forces(self):
    #     return self.F_ee, self.Tau_ee

    # def get_ee_position(self):
    #     q, t = self.transform.get_current_pose()
    #     return t

    # def set_ee_static_forces(self, F, tau):
    #     self.F_ee = F
    #     self.Tau_ee = tau

def main(args=None):
    rclpy.init(args=args)
    
    robot = Robot()

    rclpy.spin(robot)

    # robot.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()