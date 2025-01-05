import numpy as np
import math
np.set_printoptions(precision=4)


class Trajectory:
    JOINT_SPACE = "joint_space"
    CARTESIAN_SPACE = "cartasian_space"
    TRAJECTORY_TRAPEZOIDAL = 'trapezoidal'
    TRAJECTORY_CUBIC = 'cubic'
    TRAJECTORY_QUINTIC = 'quintic'
    N_BLEND_STEPS = 10
    N_MIDDLE_STEPS = 50
    delta_t = 0.001
    COLORS = ['red', 'orange', 'crimson', 'magenta', 'blue', 'limegreen']

    def __init__(self, kinematic_model, dynamic_model, dt=delta_t, trajectory_method=JOINT_SPACE, trajectory_type=TRAJECTORY_CUBIC, precision=4):
        self.delta_t = dt
        self.precision = precision
        self.round_decimals = len(str(self.delta_t).split(".")[-1])
        self.trajectory_type = trajectory_type
        self.trajectory_method = trajectory_method
        self.kinematics = kinematic_model
        self.dynamics = dynamic_model
        self.N = self.kinematics.transform.n
        self.n_steps = 2*self.N_BLEND_STEPS+self.N_MIDDLE_STEPS
        self.t = None
        self.q = None
        self.q_d = None
        self.q_dd = None
        self.cur_step = 0
        self.cur_goal = [0, 0, 0, 0]
        self.in_joint_space = []
        self.in_cartasian_space = None

    def init_goal(self):
        self.update(0)

    def is_point_accessible(self, goal_point):
        theta = self.kinematics.inverse(goal_point)

        for i in range(self.n):
            joint_limit = self.model.joint_limits[i]
            if ((joint_limit[0] <= theta[i, 0]) and theta[i, 0] <= joint_limit[1]):
                continue
            else:
                return False
        return True

    def trajectory(self):
        # print("t :", self.t)
        # print("q :", self.q)
        # print("q_d :", self.q_d)
        # print("q_dd :", self.q_dd)
        self.in_cartasian_space = np.zeros((3, self.n_steps))

        # print(self.q)

        for i in range(self.n_steps):
            self.in_cartasian_space[:, i] = self.kinematics.forward(
                self.q[:, [i]])
            # print(" Q :", self.q[:, i])
            # print(" C :", self.in_cartasian_space[:, i])
        return self

    def get_cur_goal(self):
        return self.cur_goal

    def update(self, n):
        self.cur_goal = [round(self.t[0, n], self.round_decimals),
                         self.q[:, [n]],
                         self.q_d[:, [n]],
                         self.q_dd[:, [n]]]

    def update_cur_goal(self):
        self.cur_step += 1
        if (self.cur_step >= self.n_steps):
            self.cur_step = 0
            return
        self.update(self.cur_step)

    def to_cartesian_space(self):
        return self.in_cartasian_space

    def to_joint_space(self):
        return self.in_joint_space


class TrajectoryGenerator(Trajectory):
    def __init__(self, kinematic_model, dynamic_model, delta_t=Trajectory.delta_t, trajectory_type=Trajectory.TRAJECTORY_CUBIC, joint_space=True, precision=4):
        super().__init__(kinematic_model,
                         dynamic_model,
                         delta_t,
                         trajectory_method=Trajectory.JOINT_SPACE if joint_space else Trajectory.CARTESIAN_SPACE,
                         trajectory_type=trajectory_type, precision=precision)

    def set_trajectory_gen_type(self, type):
        self.trajectory_type = type

    def generate(self, t_f, q_i, q_f, q_dot_i, q_dot_f, const_acceleration):
        self.t = np.arange(0, t_f, self.delta_t).reshape(1, -1)
        self.q = np.zeros((self.N, self.n_steps))
        self.q_d = np.zeros((self.N, self.n_steps))
        self.q_dd = np.zeros((self.N, self.n_steps))

        if self.trajectory_type == self.TRAJECTORY_TRAPEZOIDAL:
            t_blend = np.round(0.5*t_f-0.5 * np.sqrt(((const_acceleration*t_f**2) - 4*(
                q_f-q_i)) / (const_acceleration)), self.round_decimals)  # blend time

            # print('Blend times :', t_blend)
            for i in range(self.N):
                if (q_f[i] == q_i[i]):
                    continue

                n_blend = round(t_blend[i, 0]/self.delta_t)
                n_mid = round((t_f-2*t_blend[i, 0])/self.delta_t)
                n_mid_s = n_blend
                n_mid_e = n_blend + n_mid

                # print("Belnd time :", t_blend, t_blend.shape)

                # 0 <= t <= t_blend -> 1st segment position 0 to t_blend
                self.q[i, 0:n_mid_s] = np.round(
                    q_i[i, 0] + (0.5*const_acceleration[i, 0]*(self.t[0, :n_mid_s]**2)), self.precision)
                self.q_d[i, 0:n_mid_s] = const_acceleration[i, 0] * \
                    self.t[0, :n_mid_s]
                self.q_dd[i, 0:n_mid_s] = const_acceleration[i, 0] * \
                    np.ones_like(self.t[0, :n_mid_s])

                # t_blend <= t <= (t_f - t_blend)  -> 2nd segment position t_blend to t_f-t_blend
                self.q[i, n_mid_s:n_mid_e] = np.round(q_i[i, 0] + const_acceleration[i, 0] *
                                                      t_blend[i, 0]*(self.t[0, n_mid_s:n_mid_e]-0.5*t_blend[i, 0]), self.precision)
                self.q_d[i, n_mid_s:n_mid_e] = const_acceleration[i, 0] * \
                    t_blend[i, 0]*np.ones_like(self.t[0, n_mid_s:n_mid_e])
                self.q_dd[i, n_mid_s: n_mid_e] = np.zeros_like(
                    self.t[0, n_mid_s:n_mid_e])

                # (t_f - t_blend) <= t <= t_f -> 3rd segment position t_f-t_blend to t_f
                self.q[i, n_mid_e:] = np.round(
                    q_f[i] - (0.5*const_acceleration[i, 0]*((t_f-self.t[0, n_mid_e:])**2)), self.precision)
                self.q_d[i, n_mid_e:] = const_acceleration[i, 0] * \
                    (t_f-self.t[0, n_mid_e:])
                self.q_dd[i, n_mid_e:] = -const_acceleration[i, 0] * \
                    np.ones_like(self.t[0, n_mid_e:])

        elif self.trajectory_type == self.TRAJECTORY_CUBIC:
            a = q_i
            b = q_dot_i
            c = (-3 * q_i + 3 * q_f - 2 * t_f *
                 q_dot_i - t_f * q_dot_f) / t_f**2
            d = (2 * q_i - 2 * q_f + t_f *
                 q_dot_i + t_f * q_dot_f) / t_f**3

            self.q[:, :] = a + b  @ self.t + c  @ self.t**2 + d  @ self.t**3
            self.q_d[:, :] = b + 2 * c  @ self.t + 3 * d  @ self.t**2
            self.q_dd[:, :] = 2 * c + 6 * d  @ self.t

        elif self.trajectory_type == self.TRAJECTORY_QUINTIC:
            a = q_i
            b = q_dot_i
            c = (-3 * q_i + 3 * q_f - 2 * t_f *
                 q_dot_i - t_f * q_dot_f) / t_f**2
            d = (2 * q_i - 2 * q_f + t_f *
                 q_dot_i + t_f * q_dot_f) / t_f**3
            e = np.zeros_like(q_i)
            f = np.zeros_like(q_i)

            self.q[:, :] = a + b @ self.t + c @ self.t**2 + \
                d @ self.t**3 + e @ self.t**4 + f @ self.t**5
            self.q_d[:, :] = b + 2 * c @ self.t + 3 * \
                d @ self.t**2 + 4*e@self.t**3 + 5*f@self.t**4
            self.q_dd[:, :] = 2 * c + 6 * d @ \
                self.t + 12*e@self.t**2 + 20*f@self.t**3

        return self.trajectory()

    def generate_trajectory(self, intit_pt, final_pt, t_i, t_f, v_i=[], v_f=[], const_acceleration=5):
        self.n_steps = int((t_f-t_i)/self.delta_t)

        print("Inital point :", intit_pt)
        print("Final point :", final_pt)
        q_i = None
        q_f = None

        if (self.trajectory_method == Trajectory.JOINT_SPACE):
            # status_i, theta_i = self.kinematics.inverse(intit_pt)
            theta_i = self.kinematics.transform.get_current_joint_state()
            status_f, theta_f = self.kinematics.inverse(final_pt)

            print("Inital theta :", np.rad2deg(theta_i))
            print("Final theta :", np.rad2deg(theta_f))

            if (not status_f):
                return False, "Goal is not reachable!"

            q_i = theta_i
            q_dot_i = self.kinematics.get_joint_velocity(v_i)
            q_dot_f = self.kinematics.get_joint_velocity(v_f)

            for i in range(theta_f.shape[1]):
                q_f = theta_f[:, [i]]
                trajectory = self.generate(
                    t_f, q_i, q_f, q_dot_i, q_dot_f, const_acceleration)

                for i in range(trajectory.n_steps):
                    status = self.is_point_accessible(trajectory.q[:, [i]])

                    if (not status):
                        return False, "Trajectory can not be generated in this scheme!"

            return trajectory
        else:
            q_i = intit_pt
            q_f = final_pt
            q_dot_i = v_i
            q_dot_f = v_f

            trajectory = self.generate(
                t_f, q_i, q_f, q_dot_i, q_dot_f, const_acceleration)
