import numpy as np

# Model Predictive Controller


class MPC:
    pass


class Controller:
    def __init__(self, robot, th_epselon=0.01, th_d_epselon=.01, max_f_dash=10):
        self.robot = robot
        self.goal = []
        self.time_stamp = 0
        self.th_epselon = th_epselon
        self.th_d_epselon = th_d_epselon
        self.K_v = 8
        self.K_p = 16
        self.K_i = 0.75
        self.K_d = .25
        self.max_f_dash = max_f_dash
        self.err_i = np.zeros((self.robot.n, 1))
        self.pre_theta = np.zeros((self.robot.n, 1))
        self.theta_ref = np.zeros((self.robot.n, 1))
        self.theta_d_ref = np.zeros((self.robot.n, 1))
        self.theta_dd_ref = np.zeros((self.robot.n, 1))
        self.controller_running = False

    def is_goal_reached(self):
        th = self.robot.get_joint_values()
        th_e = self.goal[0]-th[0]
        th_d_e = self.goal[1]-th[1]

        if (th_e <= self.th_epselon and th_d_e <= self.th_d_epselon):
            return True
        return False

    def PID(self, theta, theta_d):
        err = self.theta_ref-theta
        self.err_i += err
        err_d = (theta - self.pre_theta)/self.robot.delta_t
        err_v = self.theta_d_ref-theta_d

        f_dash = self.theta_dd_ref + self.K_v*err_v + \
            self.K_p*err + self.K_i*self.err_i + self.K_d*err_d

        # Limiting maximum force
        f_dash[f_dash > self.max_f_dash] = self.max_f_dash
        f_dash[f_dash < -self.max_f_dash] = -self.max_f_dash

        self.pre_theta = theta
        return f_dash

    def execute(self, trajectory):
        self.controller_running = True
        # Control trajectory using PID & Control law partitioning
        for _ in range(trajectory.n_steps):
            self.goal[:] = trajectory.get_cur_goal()
            self.time_stamp = self.goal[0]

            self.theta_ref[:, :] = self.goal[1]
            self.theta_d_ref[:, :] = self.goal[2]
            self.theta_dd_ref[:, :] = self.goal[3]

            theta, theta_d = self.robot.get_joint_values()
            f_dash = self.PID(theta, theta_d)
            F = self.robot.dynamics.forward(
                theta=theta, theta_d=theta_d, theta_dd=f_dash)
            self.robot.step(F, self.time_stamp)
            trajectory.update_cur_goal()
        return True
    
    def sim_execute(self, trajectory):
        self.controller_running = True
        # Control trajectory using PID & Control law partitioning
        for _ in range(trajectory.n_steps):
            self.goal[:] = trajectory.get_cur_goal()
            self.time_stamp = self.goal[0]

            self.theta_ref[:, :] = self.goal[1]
            self.theta_d_ref[:, :] = self.goal[2]
            self.theta_dd_ref[:, :] = self.goal[3]
            self.robot.move_joint(self.goal[1])
            trajectory.update_cur_goal()
