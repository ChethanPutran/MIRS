import numpy as np
import trajectory_generator as tg
from dynamics.dynamics import Dynamics
from kinematics import Kinematics
from controller import Controller
from ai import AI
import time


class Robot:
    COLORS = ['red', 'orange', 'crimson', 'magenta', 'blue', 'limegreen']
    axis_colors = ['red', 'green', 'blue']
    movements = {
        "MOVE_UP": np.array([[0], [0], [1]]),
        "MOVE_DOWN": np.array([[0], [0], [-1]]),
        "MOVE_LEFT": np.array([[0], [1], [0]]),
        "MOVE_RIGHT": np.array([[0], [-1], [0]]),
        "MOVE_FRONT": np.array([[1], [0], [0]]),
        "MOVE_BACK": np.array([[-1], [0], [0]]),
    }

    def __init__(self, robot_model, transform, delta_t=0.1, precision=4, debug=False, simp_sim=False):
        self.delta_t = delta_t
        self.simp_sim = simp_sim
        self.model = robot_model
        self.n = robot_model.n
        self.debug = debug
        self.time_stamp = None
        self.u = 0
        self.precision = precision
        self.on_exit_callback = None

        self.theta = np.zeros((self.n, 1))
        self.theta_d = np.zeros((self.n, 1))
        self.theta_dd = np.zeros((self.n, 1))

        self.Tau_ee = np.zeros((3, 1))
        self.F_ee = np.zeros((3, 1))

        self.colors = self.COLORS[:self.n+1]
        self.links_in_frame = np.array(self.model.links)
        self.links = self.model.links

        self.controller = Controller(self)
        self.transform = transform

        self.kinematics = Kinematics(self.transform, self.precision)
        self.dynamics = Dynamics(self.transform, self.precision)
        self.trajec_gen = tg.TrajectoryGenerator(self.kinematics,
                                                 self.dynamics,
                                                 self.delta_t,
                                                 precision=self.precision,
                                                 trajectory_type=tg.Trajectory.TRAJECTORY_CUBIC)
        self.ai = AI()

        self.init_home_pos()

    def on_exit(self, callback):
        self.on_exit_callback = callback

    def get_disp_vector(self, action, displacement):
        return self.movements[action]*displacement

    def on_joint_move(self, callback):
        self.on_joint_move_callback = callback

    def on_simp_sim(self, callback):
        self.on_simp_sim_callback = callback

    def on_new_trajectory(self, callback):
        self.on_new_trajectory_callback = callback

    def init_home_pos(self):
        # Compute home position
        for i in range(0, self.n):
            # L - 2x4 -> 4x2
            self.links[i] = np.dot(self.transform._0T(i), np.dot(
                self.transform.Rot(self.theta[i, 0]), self.links_in_frame[i].T))

    def exit(self):
        print("Exiting from the program...")
        self.on_exit_callback()
        exit(0)

    def move_to_goal(self, goal_point, T, simple_sim=False):
        start_orientation, start_point = self.transform.get_current_pose()

        ons = np.ones((self.n, 1))

        status, trajectory = self.trajec_gen.generate_trajectory(
            start_point, goal_point, 0, T, 0.1*ons, 0.2*ons, 0.3*ons)

        if (not status):
            # print(trajectory)
            return

        self.on_new_trajectory_callback(trajectory)

        # self.trajec_plotter.plot_trajectory(trajectory)
        trajectory.init_goal()

        if self.simp_sim:
            self.on_simp_sim_callback(trajectory, self.move_joint)
        else:
            self.controller.execute(trajectory)

    def add_noise(self, th, noise_mean=0, noise_var=np.pi/180):
        return np.round(th + np.random.normal(noise_mean, noise_var, size=(self.n, 1)), self.precision)

    def step(self, u, time_stamp=None, add_noise=False):
        self.u = u
        self.time_stamp = time_stamp
        self.theta_dd[:, :] = self.dynamics.inverse(
            u, self.theta, self.theta_d)

        theta_d = self.theta_d + self.theta_dd * self.delta_t
        theta = self.theta + self.theta_d * \
            self.delta_t + self.theta_dd * self.delta_t**2

        if add_noise:
            theta = self.add_noise(theta)
            theta_d = self.add_noise(theta_d)

        if self.debug:
            print(f"T : {u}")
            print(f"theta :{theta}")
            print(f"theta_d :{theta_d}")
            print(f"theta_dd :{self.theta_dd}")

        self.move_joint(theta, theta_d, self.theta_dd)

    def execute_program(self, file_name="program.txt"):
        actions = ["PICK", "PLACE"]
        movements = ["MOVE_DOWN", "MOVE_UP", "MOVE_LEFT",
                     "MOVE_RIGHT", "MOVE_FRONT", "MOVE_BACK", "MOVE"]

        # print("File path :", file_name)
        command_list = []
        args_list = []

        with open(file_name, 'r') as f:
            for line in f.readlines():
                res = line.split()
                command_list.append(res[0])
                args_list.append(res[1:])

        for i in range(len(command_list)):
            print(f"Command :{command_list[i]} Args :{args_list[i]}")
            command = command_list[i]
            args = args_list[i]

            if (command in actions):
                # position = self.ai.get_object_position(args[0])
                position = np.array(args[1:4], dtype=np.float16).reshape(-1, 1)
                print(f"Action :{str.title(command)}ing object...")
                self.move_to_goal(position, int(args[-1]))
            else:
                print(f"Movement :{str.title(command)}")
                displacement_vec = self.get_disp_vector(
                    command, float(args[0]))
                print("Disp vec :", displacement_vec)
                print("Cur vec :", self.get_ee_position())
                self.move_to_goal(self.get_ee_position() +
                                  displacement_vec, float(args[1]))
            time.sleep(0.1)

    def move_joint(self, theta, theta_d=np.ones((3, 1)), theta_dd=np.ones((3, 1))):
        if self.debug:
            print("Moving Theta : ", theta, "\n\n")

        self.theta[:, :] = theta
        self.theta_d[:, :] = theta_d
        self.theta_dd[:, :] = theta_dd

        self.transform.update(theta)
        self.update_link_pos()

        if (self.controller.controller_running):
            self.on_joint_move_callback(self.time_stamp,
                                        [self.controller.theta_ref,
                                         self.theta],
                                        [self.controller.theta_d_ref,
                                         self.theta_d],
                                        self.u)
        else:
            self.on_joint_move_callback(self.time_stamp)

    def update_link_pos(self):
        for i in range(0, self.n):
            self.links[i] = np.dot(self.transform._0T(i), np.dot(self.transform.Rot(
                self.theta[i, 0]), self.links_in_frame[i].T))

    def get_joint_values(self):
        return self.theta, self.theta_d

    def get_ee_static_forces(self):
        return self.F_ee, self.Tau_ee

    def get_ee_position(self):
        q, t = self.transform.get_current_pose()
        return t

    def set_ee_static_forces(self, F, tau):
        self.F_ee = F
        self.Tau_ee = tau
