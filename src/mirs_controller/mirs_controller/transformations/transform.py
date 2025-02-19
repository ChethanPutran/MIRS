import numpy as np
import math

pi = math.pi
cos = math.cos
sin = math.sin


class Frame:
    AXIS_COLORS = ['red', 'green', 'blue']

    def __init__(self, n):
        self.n = n
        pass


class Transform:
    def __init__(self, robot_model, precision=4, frame_size=0.025, debug=False):
        self.n = robot_model.n
        self.debug = debug
        self.precision = precision
        self.robot_model = robot_model
        self.DH_PARAMS = robot_model.DH_PARAMS
        self.theta = np.zeros((self.n, 1))
        self.pre_theta = np.zeros((self.n, 1))

        self.__T = [None]*(self.n+1)
        self.__0Ti = [None]*(self.n+1)
        # (n+1)x1 vector of frames
        self.__frames = [None]*(self.n+1)

        self.frame = np.array([
            [0, frame_size*1, 0,     0, 0,     0],
            [0,    0, 0, frame_size*1, 0,     0],
            [0,    0, 0,     0, 0, frame_size*1],
            [1,    1, 1,     1, 1,     1],
        ])
        self.__T[0] = np.eye(4)
        self.__0Ti[0] = np.eye(4)

        # Initialize frames
        for i in range(0, self.n+1):
            self.__frames[i] = np.array([[0, frame_size*1, 0,     0, 0,     0],
                                         [0,    0, 0, frame_size*1, 0,     0],
                                         [0,    0, 0,     0, 0, frame_size*1],
                                         [1,    1, 1,     1, 1,     1],
                                         ])

        self.compute()

    def print(self):
        print(self.__0Ti)

    def compute_frames(self):
        for i in range(1, self.n+1):
            self.__frames[i][:] = np.dot(self._0T(i), self.frame)

    def update(self, theta):
        # print("Transform update : pre_theta", self.pre_theta)
        if self.debug:
            print("Transform update : theta", np.rad2deg(theta))
        self.theta[:, :] = theta

        self.compute()

    def DH(self, a, alpha, d, theta):
        return [[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta),  cos(theta)*cos(alpha),
                -cos(theta)*sin(alpha), a*sin(theta)],
                [0,        sin(alpha),       cos(alpha),       d],
                [0,             0,            0,       1]]

    def R(self, i):
        return self.__T[i][:3, :3]

    def T(self, i):
        return self.__T[i]

    def iTn(self, i):
        return np.linalg.inv(self.__0Ti[i]) @ self.__0Ti[self.n]

    def T(self):
        return self.__0Ti[self.n]

    def _0T(self, i):
        return self.__0Ti[i]

    def _0R(self, i):
        return self.__0Ti[i][:3, :3]

    def t(self, i):
        return self.__T[i][:3, [3]]

    def Rot(self, theta, axis='z'):
        if (axis == 'z'):
            return [[cos(theta), -sin(theta), 0, 0],
                    [sin(theta), cos(theta), 0, 0],
                    [0,     0, 1, 0],
                    [0,     0, 0, 1]]

    def H(self, arr):
        if arr.shape[1] > 1:
            H = np.eye(4)
            H[0:3, 0:3] = arr
            return H
        H = np.ones((4, 1))
        H[0:3, 0:1] = arr
        return H

    def compute(self, update=True, theta=[]):
        if (not update):
            _0Tn = np.eye(4)
            for i in range(1, self.n+1):
                _0Tn = np.dot(_0Tn, self.DH(
                    self.DH_PARAMS[i-1][0], self.DH_PARAMS[i-1][1], self.DH_PARAMS[i-1][2], theta[i-1, 0]))
            return np.round(_0Tn, self.precision)

        for i in range(1, self.n+1):
            # (i-1)Ti
            self.__T[i] = np.round(self.DH(self.DH_PARAMS[i-1][0],
                                           self.DH_PARAMS[i-1][1],
                                           self.DH_PARAMS[i-1][2],
                                           self.theta[i-1, 0]), self.precision)

            # 0Ti
            self.__0Ti[i] = np.round(
                np.dot(self.__0Ti[i-1], self.__T[i]), self.precision)
            # print(f"{i-1}T{i} :", self.__T[i])
            # print(f"0T{i} :", self.__0Ti[i])

        # Frames
        self.compute_frames()

    def rotation_matrix_to_quaternion(self, m):
        # q0 = qw
        t = np.matrix.trace(m)
        q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        if (t > 0):
            t = np.sqrt(t + 1)
            q[3] = 0.5 * t
            t = 0.5/t
            q[0] = (m[2, 1] - m[1, 2]) * t
            q[1] = (m[0, 2] - m[2, 0]) * t
            q[2] = (m[1, 0] - m[0, 1]) * t

        else:
            i = 0
            if (m[1, 1] > m[0, 0]):
                i = 1
            if (m[2, 2] > m[i, i]):
                i = 2
            j = (i+1) % 3
            k = (j+1) % 3

            t = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (m[k, j] - m[j, k]) * t
            q[j] = (m[j, i] + m[i, j]) * t
            q[k] = (m[k, i] + m[i, k]) * t

        return q

    def get_frame(self, i):
        return self.__frames[i]

    def get_current_pose(self):
        T = self._0T(self.n)
        q, t = self.rotation_matrix_to_quaternion(T[:3, :3]), T[:3, [3]]
        return [q, t]

    def get_current_joint_state(self):
        return self.theta
