import numpy as np
import math
from ..common.urdf_converter import RobotModel
pi = math.pi
cos = math.cos
sin = math.sin


class Frame:
    AXIS_COLORS = ['red', 'green', 'blue']

    def __init__(self, n):
        self.n = n
        pass


class Transform:
    def __init__(self, robot_model:RobotModel, precision=4, frame_size=0.025, debug=False):
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

    def get_transform_mat_from_dh(self, a, alpha, d, theta):
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
                _0Tn = np.dot(_0Tn, self.get_transform_mat_from_dh(
                    self.DH_PARAMS[i-1][0], self.DH_PARAMS[i-1][1], self.DH_PARAMS[i-1][2], theta[i-1, 0]))
            return np.round(_0Tn, self.precision)

        for i in range(1, self.n+1):
            # (i-1)Ti
            self.__T[i] = np.round(self.get_transform_mat_from_dh(self.DH_PARAMS[i-1][0],
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

    def rotmat_to_quaternion(self, m):
        # q0 = qw
        q = np.zeros((4,1), dtype=np.float64)
        t = np.matrix.trace(m)

        if (t > 0):
            t = np.sqrt(t + 1)
            q[3,0] = 0.5 * t
            t = 0.5/t
            q[0,0] = (m[2, 1] - m[1, 2]) * t
            q[1,0] = (m[0, 2] - m[2, 0]) * t
            q[2,0] = (m[1, 0] - m[0, 1]) * t

        else:
            i = 0
            if (m[1, 1] > m[0, 0]):
                i = 1
            if (m[2, 2] > m[i, i]):
                i = 2
            j = (i+1) % 3
            k = (j+1) % 3

            t = np.sqrt(m[i, i] - m[j, j] - m[k, k] + 1)
            q[i,0] = 0.5 * t
            t = 0.5 / t
            q[3,0]= (m[k, j] - m[j, k]) * t
            q[j,0] = (m[j, i] + m[i, j]) * t
            q[k,0] = (m[k, i] + m[i, k]) * t

        return q

    
    def quaternion_to_rotmat(self,x, y, z, w):
        rot_mat = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)]
        ])
        return rot_mat
    

    def quaternion_to_euler(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
 
    def get_frame(self, i):
        return self.__frames[i]

    def get_current_pose(self):
        T = self._0T(self.n)
        return np.concatenate((T[:3, [3]],self.rotmat_to_quaternion(T[:3, :3])),axis=0)

    def get_current_joint_state(self):
        return self.theta
