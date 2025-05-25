from mpl_toolkits.mplot3d import Axes3D
import uuid
from matplotlib.gridspec import GridSpec
import matplotlib.pyplot as plt
import numpy as np
import time
import math
import tkinter as tk
from functools import partial
from tkinter import ttk, messagebox, Frame
from sympy import *
import inspect
from M import M
from C import C
from G import G
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


UNIT = 1  # in m
DEBUG = False
FRAME_SCALE = 2
FRAME_SIZE = FRAME_SCALE*0.025*UNIT
FRAME_LINE_WIDTH = 1
N = 3
SYMBOLIC = False
HISTORY_FILE = r"C:\Chethan\Technologies\Robotics\Major_Project\mirs\src\mirs_controller\mirs_controller\trajectory\control\history.txt"
UNIT_FACTOR = 1  # in m (*1000 for unit in mm)
pi = math.pi
cos = math.cos
sin = math.sin

# 1. Markov Descision Process (MDP)
# 2. Dynamic Programming
# 3. Monte Carlo Methods
# 4. Temporal Difference Methods
# 5. N-step Bootstraping
# 6. Continuous state spaces
# 7. Deep SARSA
# 8. Deep Q-Learning
# 9. Actor Critic (A2C)

# 3. Monte Carlo Methods

# i) On-Policy Monte Carlo




def get_coordinates(mat):
    X = mat[0, :]
    Y = mat[1, :]
    Z = mat[2, :]
    return X, Y, Z


class Envirmonment:
    def __init__(self, size=5) -> None:
        self.x = np.random.randint(0, size-1)
        self.y = np.random.randint(0, size-1)
        self.goal = (size, size)
        self.actions = [1, 2, 3, 4]
        self.act = {
            1: (-1, 0),  # Left
            2: (0, 1),  # Right
            3: (-1, 0),  # Top
            4: (0, 1),  # Bottom
        }
        self.n_actions = 4

    # Reward,state,episode_end
    def step(self, action):
        temp_x = self.x + self.act[action][0]
        temp_y = self.y + self.act[action][1]

        if (temp_x >= self.size) or (temp_y >= self.size) or (temp_x < 0) or (temp_y < 0):
            return -1, (self.x, self.y), False
        if (temp_x == (self.size-1) and temp_y == (self.size-1)):
            return 1, (self.x, self.y), True

env = Envirmonment()

class AI:
    def __init__(self):
        pass

    def get_object_position(self, object):
        # return np.array([0.4330, 0, 0]).reshape(-1, 1)
        return np.array([0, 0.55, 0.2]).reshape(-1, 1)
    

class RobotModel:
    def __init__(self, n=3, unit=UNIT_FACTOR):
        # Number of DOFs
        self.n = n
        self.unit = unit

        # Length along two Z axces LINK_
        # Length between two z axces LINK_T
        # LINK_0_LENGTH = 0
        # LINK_1_LENGTH = 100
        # LINK_T_2_LENGTH = 70
        # LINK_2_LENGTH = 250
        # LINK_3_LENGTH = 250
        # LINK_T_4_LENGTH = 70
        # LINK_T_5_LENGTH = 70
        # LINK_5_LENGTH = 50
        # LINK_6_LENGTH = 60

        # Link parameters (in m)
        # Link length

        l0 = 0.2*unit
        l1 = 0.3*unit
        l2 = 0.25*unit

        self.H_MAX = l0+l1+l2+(.025*unit)

        self.link_lengths = [l0, l1, l2]

        # Link radii
        # L0
        d0 = 0.06*unit
        r0 = 0.03*unit

        # L1
        d1 = 0.05*unit
        r1 = 0.025*unit

        # L2
        d2 = 0.04*unit
        r2 = 0.02*unit

        # L2
        # Link density (in kg/m^3)
        rho = 2710/(unit**3)

        # nx2x4
        self.links = [
            [[0, 0, 0, 1], [0, 0, l0, 1]],
            [[0, 0, 0, 1], [l1, 0, 0, 1]],
            [[0, 0, 0, 1], [l2, 0, 0, 1]],
        ]
        self.joint_limits = [(0, 360), (-45, 225), (-45, 225)]
        # DH parameters
        # nx4 matrix of DH parameters
        self.DH_PARAMS = [[0, math.pi/2, l0, 0],
                          [l1, 0, 0, 0],
                          [l2, 0, 0, 0]]

        # Link masses (in Kg)
        m0 = rho*(0.25*math.pi*d0**2*l0)
        m1 = rho*(0.25*math.pi*d1**2*l1)
        m2 = rho*(0.25*math.pi*d2**2*l2)

        self.link_masses = [m0, m1, m2]

        # Link Inertias (in Kgm^2)
        I0xx = 0.25*m0*r0**2 + 0.333*m0*l0**2  # At end
        I0yy = 0.25*m0*r0**2 + 0.33*m0*l0**2  # At end
        I0zz = 0.5*m0*r0**2  # At axis
        I0xy = 0
        I0xz = 0
        I0yz = 0

        I1xx = 0.5*m1*r1**2  # At axis
        I1yy = 0.25*m1*r1**2 + 0.333*m1*l1**2  # At end
        I1zz = 0.25*m1*r1**2 + 0.333*m1*l1**2  # At end
        I1xy = 0
        I1xz = 0
        I1yz = 0

        I2xx = 0.5*m2*r2**2  # At axis
        I2yy = 0.25*m2*r2**2 + 0.333*m2*l2**2  # At end
        I2zz = 0.25*m2*r2**2 + 0.333*m2*l2**2  # At end
        I2xy = 0
        I2xz = 0
        I2yz = 0
        self.r_bar = np.array([[0, 0, l0/2, 1],
                               [l1/2, 0, 0, 1],
                               [l2/2, 0, 0, 1]]).T
        self.link_inertias = [[I0xx, I0yy, I0zz, I0xy, I0xz, I0yz],
                              [I1xx, I1yy, I1zz, I1xy, I1xz, I1yz],
                              [I2xx, I2yy, I2zz, I2xy, I2xz, I2yz]]


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
        logger.debug(self.__0Ti)

    def compute_frames(self):
        for i in range(1, self.n+1):
            self.__frames[i][:] = np.dot(self._0T(i), self.frame)

    def update(self, theta):
        # logger.debug("Transform update : pre_theta", self.pre_theta)
        if self.debug:
           logger.debug("Transform update : theta", np.rad2deg(theta))
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
            # logger.debug(f"{i-1}T{i} :", self.__T[i])
            # logger.debug(f"0T{i} :", self.__0Ti[i])

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


class Kinematics:
    def __init__(self, transform:Transform, precision=4):
        self.precision = precision
        self.transform = transform
        self.J = np.zeros((6, self.transform.n))
        self.V = np.zeros((3, 1))
        self.W = np.zeros((3, 1))
        self.Q = np.zeros((4, 4))
        self.Q[1, 0] = 1
        self.Q[0, 1] = -1

        # iWi -> unit angular rotation vector
        self.iWi = np.zeros((3, 1))
        self.iWi[2, 0] = 1

    def delT_delq(self, i):
        Q = np.zeros((4, 4))
        Q[1, 0] = 1
        Q[0, 1] = -1
        return self.transform._0T(i-1) @ Q @ self.transform.iTn(i-1)

    def Vx(self, skew_mat):
        s = np.zeros((3, 1))
        s[0, 0] = skew_mat[2, 1]
        s[1, 0] = skew_mat[0, 2]
        s[2, 0] = skew_mat[1, 0]
        return s

    def rho(self, rot_mat):
        return rot_mat[:3, :3]

    def tau(self, rot_mat):
        return rot_mat[:3, [3]]

    def get_ee_velocity(self, q_dot):
        return self.jacobian() @ q_dot

    def get_joint_velocity(self, v:np.ndarray):
        J = self.jacobian()
        J_psuedo = np.linalg.inv(J.T @ J) @ J.T

        logger.debug(J_psuedo)
        logger.debug(v)
        return J_psuedo @ v

    def jacobian(self):

        # ∨× -> Maps a skew symmetric matrix to a vector
        # ρ() : Rotation component extractionfunction
        # τ() : Translation component extractionfunction
        # J_ωj(q) = ∨×(ρ(∂T(q)/∂qj)*ρ(T(q).T))
        # J_vj(q) = τ(∂T(q)/∂qj)
        # iV_i+1 = iVi + iWi x iP_i+1
        # iW_i+1 = iWi + iR_i+1 . theta_d_i+1 . i+1_Z_i+1

        for i in range(1, self.transform.n+1):
            # Jv
            self.J[:3, [i-1]] = self.tau(self.transform._0T(i-1) @
                                         self.Q @ self.transform.iTn(i-1))
            # Jw
            self.J[3:, [i-1]] = self.Vx(self.rho(self.transform._0T(i-1)) @
                                        self.rho(self.Q) @ self.rho(self.transform._0T(i-1).T))

        return self.J

    def jacobian2(self):

        # ∨× -> Maps a skew symmetric matrix to a vector
        # ρ() : Rotation component extractionfunction
        # τ() : Translation component extractionfunction
        # J_ωj(q) = ∨×(ρ(∂T(q)/∂qj)*ρ(T(q).T))
        # J_vj(q) = τ(∂T(q)/∂qj)
        # iV_i+1 = iVi + iWi x iP_i+1
        # iW_i+1 = iWi + iR_i+1 . theta_d_i+1 . i+1_Z_i+1

        for i in range(1, self.transform.n+1):

            # del(T)/del(q_i)
            delT_delqi = self.delT_delq(i)
            # Jv
            self.J[:3, [i-1]] = delT_delqi[:3, [3]]

            # Jw
            self.J[3:, [i-1]] = self.Vx(
                delT_delqi[:3, :3] @ self.transform._0T(robot.n).T[:3, :3])

        return self.J

    def forward(self, theta):
        T = self.transform.compute(update=False, theta=theta)
        return T[:3, 3]

    def get_transform(self):
        return self.transform

    def inverse(self, pt, deg=False):
        try:
            theta = np.zeros((self.transform.n, 2))
            x, y, z = pt
            l0, l1, l2 = self.transform.robot_model.link_lengths
            theta1 = math.atan2(y, x)
            # logger.debug("Theta1 :", theta1)
            th3_val = np.round(
                (x**2 + y**2 + (z-l0)**2 - l1**2 - l2**2)/(2*l1*l2), 4)
            # logger.debug("th3_val :", th3_val)

            if (th3_val > 1):
                logger.debug("Goal is out of reach of robot")
                return False, []

            theta3 = np.round(math.acos(th3_val), self.precision)
            theta21 = np.round(math.atan2(z-l0, math.sqrt(x**2+y**2)) -
                            math.atan2(math.sin(theta3)*l2, (l1+math.cos(theta3)*l2)), self.precision)
            theta22 = np.round(math.atan2(z-l0, math.sqrt(x**2+y**2)) -
                            math.atan2(math.sin(-theta3)*l2, (l1+math.cos(-theta3)*l2)), self.precision)

            if deg:
                theta1 = math.degrees(theta1)
                theta21 = math.degrees(theta21)
                theta22 = math.degrees(theta22)
                theta3 = math.degrees(theta3)
            theta[:, 0] = [theta1, theta21, theta3]
            theta[:, 1] = [theta1, theta22, theta3]

            return True, theta
        except ValueError as e:
            return False, "Goal is out of reach of robot"


class Robot:
    def __init__(self, n=3):
        # Number of DOFs
        self.n = n

        # Link parameters (in m)
        # Link length
        l0 = 0.50
        l1 = 1.00
        l2 = 0.75

        # Link radii
        # L0
        d0 = 0.06
        r0 = 0.03

        # L1
        d1 = 0.05
        r1 = 0.025

        # L2
        d2 = 0.04
        r2 = 0.02

        # L2
        # Link density (in kg/m^3)
        rho = 2710

        # DH parameters
        # nx4 matrix of DH parameters
        self.DH_PARAMS = [[0, math.pi/2, l0, 0],
                          [l1, 0, 0, 0],
                          [l2, 0, 0, 0]]

        # Link masses (in Kg)
        m0 = rho*(0.25*math.pi*d0**2*l0)
        m1 = rho*(0.25*math.pi*d1**2*l1)
        m2 = rho*(0.25*math.pi*d2**2*l2)

        self.link_masses = [m0, m1, m2]

        # Link Inertias (in Kgm^2)
        I0xx = 0.25*m0*r0**2 + 0.333*m0*l0**2  # At end
        I0yy = 0.25*m0*r0**2 + 0.33*m0*l0**2  # At end
        I0zz = 0.5*m0*r0**2  # At axis
        I0xy = 0
        I0xz = 0
        I0yz = 0

        I1xx = 0.5*m1*r1**2  # At axis
        I1yy = 0.25*m1*r1**2 + 0.333*m1*l1**2  # At end
        I1zz = 0.25*m1*r1**2 + 0.333*m1*l1**2  # At end
        I1xy = 0
        I1xz = 0
        I1yz = 0

        I2xx = 0.5*m2*r2**2  # At axis
        I2yy = 0.25*m2*r2**2 + 0.333*m2*l2**2  # At end
        I2zz = 0.25*m2*r2**2 + 0.333*m2*l2**2  # At end
        I2xy = 0
        I2xz = 0
        I2yz = 0
        self.r_bar = np.array([[0, 0, l0/2, 1],
                               [l1/2, 0, 0, 1],
                               [l2/2, 0, 0, 1]]).T
        self.link_inertias = [[I0xx, I0yy, I0zz, I0xy, I0xz, I0yz],
                              [I1xx, I1yy, I1zz, I1xy, I1xz, I1yz],
                              [I2xx, I2yy, I2zz, I2xy, I2xz, I2yz]]


class DynamicsEquationGenerator:
    def __init__(self, robot):
        # Robot structure
        self.robot = robot

        # No. DOFs
        self.n = self.robot.n

        # 1xn row vector of link masses
        self.m = Array(self.robot.link_masses)

        self.r_bar = self.robot.r_bar

        # 1xn vector of 4x4 inertia matrices
        self.J = [self.inertia_tensor(
            self.robot.link_inertias[i], self.m[i], self.robot.r_bar[:, i]) for i in range(self.n)]

        # nx1 vector of joint torque / generalized coordinates (DH parameter theta)
        self.tau = Matrix([[t_] for t_ in symbols(f'tau:{self.n}')])

        # nx1 vector of joint variable / generalized coordinates (DH parameter theta)
        self.q = Matrix([[q_] for q_ in symbols(f'q:{self.n}')])

        # nx1 vector of joint velocity
        self.q_d = Matrix([[q_] for q_ in symbols(f'q_d:{self.n}')])

        # nx1 vector of joint acceleration
        self.q_dd = Matrix([[q_] for q_ in symbols(f'q_dd:{self.n}')])

        # Transformation matrices (n)x(n+1)x4x4
        self.T = [[None for _ in range(self.n+1)] for i in range(self.n)]

        # 1xn gravity row vector
        self.g = [0, 0, -9.81, 0]

        # nxn matrix represents the effect of motion of joint j on all the point of link i (U_ij ~= del(0Ai)/del(q_i))
        # U = [ U_ij : 0A(j-1) * Q_j * (j-1)Ai if j<= i else 0 ]
        # For prismatic joint (Q used for calculating the partial derivative of 0Ai w.r.to q_i)
        # Q = [[0,0,0,0],
        #      [0,0,0,0],
        #      [0,0,0,1],
        #      [0,0,0,0]]

        # For revolute joint
        self.Q_k = self.Q_j = [[0, -1, 0, 0],
                               [1, 0, 0, 0],
                               [0, 0, 0, 0],
                               [0, 0, 0, 0]]

        self.U = [[None for _ in range(self.n)] for _ in range(self.n)]

        # Interraction between joints
        # nxnxn matrix represents the effect of motion of joint j & joint k on all points on link i (U_ijk ~= del(U_ij)/del(q_k))
        # U3 = [ U_ijk : 0A(j-1) * Q_j * (j-1)A(k-1) * Q_k * (k-1)Ai if j<=k<=i else 0A(k-1) * Q_k * (k-1)A(j-1) * Q_j * (j-1)Ai if k<=j<=i else 0 ]

        self.U3 = [[[None for _ in range(self.n)]
                    for _ in range(self.n)] for _ in range(self.n)]

        # Initialize nxn inertial acceleration related symmetric matrix
        # M = M(q)
        self.M = zeros(self.n, self.n)

        # Initialize nx1 nonlinear Coriolis and centrifugal force vector
        # C = C(q,q_d)
        self.C = zeros(self.n, 1)

        # Initialize nx1 gravity loading force vector
        self.G = zeros(self.n, 1)

        self.generate_trnasformation_matrices()
        self.generate_partial_derivative_matrices()
        self.generate_M_matrix()
        self.generate_C_vector()
        self.generate_G_vector()

    def generate_partial_derivative_matrices(self):
        # Generate U2
        for i in range(1, self.n+1):
            for j in range(1, self.n+1):
                if (i < j):
                    self.U[i-1][j-1] = np.zeros((4, 4))
                else:
                    self.U[i-1][j-1] = self.T[0][j -
                                                 1] @ self.Q_j @ self.T[j-1][i]

        # Generate U3
        for i in range(1, self.n+1):
            for j in range(1, self.n+1):
                for k in range(1, self.n+1):
                    if (j <= k and k <= i):
                        self.U3[i-1][j-1][k-1] = self.T[0][j -
                                                           1] @ self.Q_j @ self.T[j-1][k-1] @ self.Q_k @ self.T[k-1][i]
                    elif (k <= j and j <= i):
                        self.U3[i-1][j-1][k-1] = self.T[0][k -
                                                           1] @ self.Q_k @ self.T[k-1][j-1] @ self.Q_j @ self.T[j-1][i]
                    else:
                        self.U3[i-1][j-1][k-1] = np.zeros((4, 4))

    def generate_M_matrix(self):
        # Generate M matrix
        for i in range(self.n):
            for k in range(self.n):
                for j in range(max(i, k), self.n):
                    self.M[i, k] = self.M[i, k] + np.trace(self.U[j]
                                                           [k] @ self.J[j] @ self.U[j][i].T)

    def generate_C_vector(self):
        # nxnxn tensor
        H = np.zeros((self.n, self.n, self.n), dtype='object')

        # Generate h_q_qd vector
        for i in range(self.n):
            for k in range(self.n):
                for m in range(self.n):
                    for j in range(max(i, k, m), self.n):
                        H[i][k][m] = H[i][k][m] + \
                            np.trace(self.U3[j][k][m] @
                                     self.J[j] @ self.U[j][i].T)    # 4x4 * 4x4 * 4x4
            self.C[i, 0] = self.q_d.T @ H[i] @ self.q_d

    def generate_G_vector(self):
        # Generate G
        for i in range(self.n):
            for j in range(i, self.n):
                self.G[i, 0] = self.G[i, 0] - self.m[j] * \
                    (self.g @ self.U[j][i] @ self.r_bar[:,
                     j].reshape(-1, 1))   # 1x4 * 4x4 * 4*1

    def generate_trnasformation_matrices(self):
        # Generate T matrix
        for i in range(self.n-1, -1, -1):  # max 0Tn
            for j in range(i, self.n+1):
                if (i == j):
                    self.T[i][j] = np.eye(4)
                elif i == (j-1):
                    self.T[i][j] = self.DH(self.robot.DH_PARAMS[i][0],
                                           self.robot.DH_PARAMS[i][1],
                                           self.robot.DH_PARAMS[i][2],
                                           self.q[i])
                else:
                    self.T[i][j] = self.T[i][j-1] @ self.T[i+1][j]

    def DH(self, a, alpha, d, theta, type='revolute'):
        if (type == 'revolute'):
            return np.array([[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
                            [sin(theta),  cos(theta)*cos(alpha), -
                             cos(theta)*sin(alpha), a*sin(theta)],
                            [0,        sin(alpha),       cos(alpha),       d],
                            [0,             0,            0,       1]])

    def inertia_tensor(self, inertia_vec, m, X):
        x_bar, y_bar, z_bar, _ = X.T
        Ixx, Iyy, Izz, Ixy, Ixz, Iyz = inertia_vec

        return np.array([[(-Ixx+Iyy+Izz)*0.5, Ixy, Ixz, m*x_bar],
                        [Ixy, (Ixx-Iyy+Izz)*0.5, Iyz, m*y_bar],
                        [Ixz, Iyz, (Ixx+Iyy-Izz)*0.5, m*z_bar],
                        [m*x_bar, m*y_bar, m*z_bar, m]])

    def generate_dynamic_param_functions(self):
        # Generate M(q) matrix
        with open("M.py", 'w') as f:
            M_func = lambdify(self.q, self.M, modules='numpy')
            f.write(inspect.getsource(M_func))

        # Generate C(q,q_d) vector
        with open("C.py", 'w') as f:
            C_func = lambdify((self.q, self.q_d), self.C, modules='numpy')
            f.write(inspect.getsource(C_func))

        # Generate G(q) vector
        with open("G.py", 'w') as f:
            G_func = lambdify(self.q, self.G, modules='numpy')
            f.write(inspect.getsource(G_func))

    def generate_inverse_dynamic_eq(self):
        logger.debug(self.M.inv())
        return
        q_dd = self.M.inv() @ (self.tau - self.C - self.G)
        q_dd_func = lambdify((self.q, self.q_d, self.tau),
                             q_dd, modules='numpy')
        return q_dd_func

    
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
            tic = time.perf_counter()
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
            toc = time.perf_counter()
            print(f"Running time in {toc - tic:0.4f} seconds")

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


class Dynamics:
    def __init__(self, transform, precision=4):
        self.M = M
        self.G = G
        self.C = C
        self.transform = transform
        self.n = transform.n
        self.Zi = np.zeros((3, 1))
        self.Zi[2, 0] = 1
        self.tau = np.zeros((self.n, 1))
        self.n_tau = np.zeros((3, self.n+1))
        self.F = np.zeros((3, self.n+1))
        self.precision = precision

    def forward(self, theta, theta_d, theta_dd):
        return np.round(self.M(*theta.ravel()) @ theta_dd + self.C(theta.ravel(), theta_d.ravel()) + self.G(*theta.ravel()), self.precision)

    def inverse(self, tau, theta0, theta_d0):
        return np.round(np.linalg.inv(self.M(*theta0.ravel())) @  (tau - self.C(theta0.ravel(), theta_d0.ravel()) - self.G(*theta0.ravel())), self.precision)

    def jacobian(self, F_ee, Tau_ee):
        # i_F_i
        self.F[:, [self.n]] = F_ee
        # i_n_i
        self.n_tau[:, [self.n]] = Tau_ee

        for i in range(self.n-1, -1, -1):
            self.F[:, [i]] = self.transform.R(i+1) @ self.F[:, [i+1]]

            logger.debug(self.transform.t(i+1))
            logger.debug(self.F[:, [i]])
            self.n_tau[:, [i]] = self.transform.R(i+1) @ self.n_tau[:, [i+1]] +\
                np.cross(self.transform.t(i+1), self.F[:, [i]], axis=0)

            # Calculate joint resisting torque (dot product between torque vector and joint axis)
            self.tau[i, 0] = self.n_tau[:, [i]].T @ self.Zi
        return self.F, self.tau


class Trajectory:
    JOINT_SPACE = "joint_space"
    CARTESIAN_SPACE = "cartasian_space"
    TRAJECTORY_TRAPEZOIDAL = 'trapezoidal'
    TRAJECTORY_CUBIC = 'cubic'
    TRAJECTORY_QUINTIC = 'quintic'
    N_BLEND_STEPS = 10
    N_MIDDLE_STEPS = 50
    DELTA_T = 0.001
    COLORS = ['red', 'orange', 'crimson', 'magenta', 'blue', 'limegreen']

    def __init__(self, kinematic_model:Kinematics, dynamic_model:Dynamics, dt=DELTA_T, trajectory_method=JOINT_SPACE, trajectory_type=TRAJECTORY_CUBIC, precision=4):
        self.delta_t = dt
        self.precision = precision
        self.round_decimals = len(str(self.delta_t).split(".")[-1])
        self.trajectory_type = trajectory_type
        self.trajectory_method = trajectory_method
        self.kinematics = kinematic_model
        self.model = self.kinematics.transform.robot_model
        self.dynamics = dynamic_model
        self.N = self.kinematics.transform.n
        self.n_steps = None
        # self.n_steps = 2*self.N_BLEND_STEPS+self.N_MIDDLE_STEPS
        self.t:np.ndarray = None
        self.q:np.ndarray = None
        self.q_d:np.ndarray = None
        self.q_dd:np.ndarray = None
        self.in_cartasian_space:np.ndarray = None
        self.in_joint_space:np.ndarray = None
        self.cur_step = 0
        self.cur_goal = [0, 0, 0, 0]

    def get_no_variables(self):
        return self.N
    
    def init_goal(self):
        self.update(0)

    def is_point_accessible(self,point_type, goal_point):
        if point_type == Trajectory.CARTESIAN_SPACE:
            status,theta = self.kinematics.inverse(goal_point)
            if not status:
                return False
        
        theta = goal_point
        logger.debug(theta)

        for i in range(self.N):
            joint_limit = self.model.joint_limits[i]
            if ((joint_limit[0] <= theta[i, 0]) and theta[i, 0] <= joint_limit[1]):
                continue
            else:
                return False
        return True

    def trajectory(self):
        self.in_cartasian_space = np.zeros((self.n_steps,self.N ))
        self.in_joint_space = np.zeros((self.n_steps,self.N ))

        if self.trajectory_method==self.JOINT_SPACE:
            self.in_joint_space[:,:] = self.q.T
            for i in range(self.n_steps):
                self.in_cartasian_space[i, :] = self.kinematics.forward(self.q[:, [i]])
        else:
            self.in_cartasian_space[:,:] = self.q.T
            for i in range(self.n_steps):
                self.in_joint_space[i, :] = self.kinematics.inverse(self.q[:, [i]])

        return self

    def get_cur_goal(self):
        return self.cur_goal

    def get_ref_variable_data(self,idx):
        if idx > self.N:
            raise "Idx should be less than N"
        return self.q[[idx],:].T
    
    def get_joint_variable_data(self):
        return self.q[:,:]

    def get_time_stamps(self):
        return self.t
    
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

    def get_instance_no(self,time_stamp):
        return time_stamp//self.delta_t
    

class TrajectoryGenerator(Trajectory):
    def __init__(self, kinematic_model, dynamic_model, delta_t=Trajectory.DELTA_T, trajectory_type=Trajectory.TRAJECTORY_CUBIC, joint_space=True, precision=4):
        super().__init__(kinematic_model,
                         dynamic_model,
                         delta_t,
                         trajectory_method=Trajectory.JOINT_SPACE if joint_space else Trajectory.CARTESIAN_SPACE,
                         trajectory_type=trajectory_type, precision=precision)

    def set_trajectory_gen_type(self, type):
        self.trajectory_type = type

    def generate(self, t_f, q_i, q_f, q_dot_i, q_dot_f, const_acceleration):
        self.t = np.arange(0, t_f, self.delta_t).reshape(1, -1)
        self.n_steps = self.t.shape[1]
        self.q = np.zeros((self.N, self.n_steps))
        self.q_d = np.zeros((self.N, self.n_steps))
        self.q_dd = np.zeros((self.N, self.n_steps))
     
        if self.trajectory_type == self.TRAJECTORY_TRAPEZOIDAL:
            t_blend = np.round(0.5*t_f-0.5 * np.sqrt(((const_acceleration*t_f**2) - 4*(
                q_f-q_i)) / (const_acceleration)), self.round_decimals)  # blend time

            # logger.debug('Blend times :', t_blend)
            for i in range(self.N):
                if (q_f[i] == q_i[i]):
                    continue

                n_blend = round(t_blend[i, 0]/self.delta_t)
                n_mid = round((t_f-2*t_blend[i, 0])/self.delta_t)
                n_mid_s = n_blend
                n_mid_e = n_blend + n_mid

                # logger.debug("Belnd time :", t_blend, t_blend.shape)

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

    def generate_trajectory(self, intit_pt, final_pt, t_i, t_f, v_i=[0.1,0.1,0.1,0.1,0.1,0.1], v_f=[0.2,0.2,0.2,0.2,0.2,0.2], const_acceleration=5):
        self.n_steps = int((t_f-t_i)/self.delta_t)

        logger.debug("Inital point :", intit_pt)
        logger.debug("Final point :", final_pt)

        q_i = None
        q_f = None

        if (self.trajectory_method == Trajectory.JOINT_SPACE):
            # status_i, theta_i = self.kinematics.inverse(intit_pt)
            theta_i = self.kinematics.transform.get_current_joint_state()
            status_f, theta_f = self.kinematics.inverse(final_pt)

            logger.debug("Inital theta :", np.rad2deg(theta_i))
            logger.debug("Final theta :", np.rad2deg(theta_f))

            if (not status_f):
                return False, "Goal is not reachable!"

            q_i = theta_i
            q_dot_i = self.kinematics.get_joint_velocity(v_i)
            q_dot_f = self.kinematics.get_joint_velocity(v_f)

            flag = True
            msg = "Trajectory can not be generated in this scheme!"
            for i in range(theta_f.shape[1]):
                q_f = theta_f[:, [i]]
                trajectory = self.generate(
                    t_f, q_i, q_f, q_dot_i, q_dot_f, const_acceleration)

                for i in range(trajectory.n_steps):
                    status = self.is_point_accessible(Trajectory.JOINT_SPACE, trajectory.q[:, [i]])

                    if (not status):
                        flag=False
                        break
                    
                if not flag:
                    return False, "Trajectory can not be generated in this scheme!"
                
                return True,trajectory
        else:
            q_i = intit_pt
            q_f = final_pt
            q_dot_i = v_i
            q_dot_f = v_f

            trajectory = self.generate(
                t_f, q_i, q_f, q_dot_i, q_dot_f, const_acceleration)
            
            for i in range(trajectory.n_steps):
                status = self.is_point_accessible(Trajectory.CARTESIAN_SPACE, trajectory.q[:, [i]])

                if (not status):
                    flag=False
                    break
                
            if not flag:
                return False, "Trajectory can not be generated in this scheme!"
            
            return True,trajectory
            
        return True,trajectory


class Frame:
    AXIS_COLORS = ['red', 'green', 'blue']

    def __init__(self, n):
        self.n = n
        pass


class PARAMS:
    GOAL_X = "goal_x"
    GOAL_Y = "goal_y"
    GOAL_Z = "goal_z"
    GOAL_T = "goal_t"


class Console:

    def __init__(self,robot, width=600, height=600, precision=4):
        self.robot = robot
        self.n = robot.get_dof()
        self.theta = np.zeros((self.n, 1))
        self.theta_dot = np.zeros((self.n, 1))
        self.joints = [None]* self.n
        self.joints_label = [None]* self.n
        self.joints_value_label = [None]* self.n
        self.joints_slider = [None]* self.n
        self.params = {
            PARAMS.GOAL_X: 0.0,
            PARAMS.GOAL_Y: 0.0,
            PARAMS.GOAL_Z: 0.0,
            PARAMS.GOAL_T: 0.0,
        }

        # root window
        self.root = tk.Tk()
        # self.root.geometry('900x300')
        self.root.resizable(False, False)
        self.root.title('MIRS Console')
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=3)
        self.window = self.root
        # self.window = Frame(self.root)
        # self.window = Frame(self.root, width=width, height=height)
        # self.window.grid(row=0, column=0, padx=(10, 10), pady=(10, 10))

        self.load_saved_params()
        self.create_ui()

    # def add_canvas_to_window(self, figure, row=5, col=0, rowspan=7, colspan=7):
    #     self.canvas = FigureCanvasTkAgg(figure, master=self.window)
    #     self.canvas.get_tk_widget().grid(
    #         row=row, column=col, columnspan=colspan, rowspan=rowspan)
    #     self.canvas.draw()

    def get_graphic_canvas(self):
        return self.canvas

    def get_root(self):
        return self.root

    def get_window(self):
        return self.window

    def move_to_goal(self):
        x = self.goal_x.get()
        y = self.goal_y.get()
        z = self.goal_z.get()
        t = self.goal_t.get()

        self.params[PARAMS.GOAL_X] = x
        self.params[PARAMS.GOAL_Y] = y
        self.params[PARAMS.GOAL_Z] = z
        self.params[PARAMS.GOAL_T] = t

        self.goal_callback(np.array([x, y, z]).reshape(-1, 1), t)

    def set_robot_params(self, robot):
        self.robot = robot

    def load_saved_params(self):
        with open(HISTORY_FILE, 'r') as f:
            for line in f.readlines():
                param = line.strip().split("=")
                param_name = param[0]
                param_value = param[1]
                self.params[param_name] = float(param_value)

    def save_params(self):
        with open(HISTORY_FILE, 'w') as f:
            for param in self.params:
                line = param+"="+str(self.params[param])+"\n"
                f.write(line)

    def before_exit(self, save_params=True):
        if save_params:
            self.save_params()

        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.root.withdraw()
            self.root.destroy()

        if self.on_exit_callback is None:
            pass
        else:
            self.on_exit_callback()

    def on_new_program(self, callback):
        self.new_program_callback = callback

    def on_goal(self, callback):
        self.goal_callback = callback

    def on_manual_control(self, callback):
        self.manual_control_callback = callback

    def on_exit(self, callback):
        self.on_exit_callback = callback

    def on_state_change(self, get_new_state):
        self.get_new_state = get_new_state

    def get_joint_value(self, i, deg=True):
        if deg:
            return self.joints[i].get()
        return np.deg2rad(self.joints[i].get())

    def slider_change_callback(self, *params):
        i = params[0]
        self.theta[i, 0] = self.get_joint_value(i, deg=False)
        self.joints_value_label[i].configure(text=round(self.theta[i, 0], 2))
        self.manual_control_callback(self.theta)
        self.set_endpoint_label()

    def select_file(self):
        # file_path = filedialog.askopenfilename(title="Select a file", filetypes=[
        #                                        ("Text files", "*.txt"), ("All files", "*.*")])
        self.new_program_callback()

    def create_ui(self):
        # Create joint variable, joint slider & label
        for i in range(0, self.n):
            # Joint
            self.joints[i] = tk.DoubleVar()

            # Joint Label
            self.joints_label[i] = ttk.Label(self.window, text=f'Joint {i+1}:')
            self.joints_label[i].grid(row=i, column=0, sticky='w')

            # Joint Slider
            self.joints_slider[i] = ttk.Scale(self.window, from_=self.robot.model.joint_limits[i][0], to=self.robot.model.joint_limits[i][1], orient='horizontal',
                                              variable=self.joints[i], command=partial(self.slider_change_callback, i))
            self.joints_slider[i].grid(row=i, column=1, sticky='we')

            # Joint Value Label
            self.joints_value_label[i] = ttk.Label(
                self.window, text=self.get_joint_value(i), width=15, anchor='center')
            self.joints_value_label[i].grid(row=i, column=2)
        
        self.ee_point_label = ttk.Label(self.window)
        self.ee_point_label.grid(row=self.n+1, column=0, sticky='w',columnspan=2)
        self.set_endpoint_label()

        self.goal_x = tk.DoubleVar()
        self.goal_y = tk.DoubleVar()
        self.goal_z = tk.DoubleVar()
        self.goal_t = tk.DoubleVar()

        # Set loaded params
        self.goal_x.set(self.params[PARAMS.GOAL_X])
        self.goal_y.set(self.params[PARAMS.GOAL_Y])
        self.goal_z.set(self.params[PARAMS.GOAL_Z])
        self.goal_t.set(self.params[PARAMS.GOAL_T])

        goal_label = ttk.Label(self.window, text='Goal')
        goal_label.grid(row=0, column=3, sticky='w')
        goal_x_label = ttk.Label(self.window, text='X :')
        goal_x_label.grid(row=1, column=3, sticky='w')
        goal_x_entry = ttk.Entry(self.window, textvariable=self.goal_x)
        goal_x_entry.grid(row=1, column=4, sticky='w')

        goal_y_label = ttk.Label(self.window, text='Y :')
        goal_y_label.grid(row=2, column=3, sticky='w')
        goal_y_entry = ttk.Entry(self.window, textvariable=self.goal_y)
        goal_y_entry.grid(row=2, column=4, sticky='w')

        goal_z_label = ttk.Label(self.window, text='Z :')
        goal_z_label.grid(row=3, column=3, sticky='w')
        goal_z_entry = ttk.Entry(self.window, textvariable=self.goal_z)
        goal_z_entry.grid(row=3, column=4, sticky='w')

        goal_t_label = ttk.Label(self.window, text='T :')
        goal_t_label.grid(row=4, column=3, sticky='w')
        goal_t_entry = ttk.Entry(self.window, textvariable=self.goal_t)
        goal_t_entry.grid(row=4, column=4, sticky='w')

        # Move to goal button
        goal_button = ttk.Button(
            self.window, text="Move", command=partial(self.move_to_goal))
        goal_button.grid(row=1, column=5)

        # Program select button
        program_select_button = ttk.Button(
            self.window, text="Execute Program", command=partial(self.select_file))
        program_select_button.grid(row=2, column=5)

        # Exit button
        exit_button = ttk.Button(
            self.window, text="Exit", command=partial(self.before_exit))
        exit_button.grid(row=3, column=5)

    def set_endpoint_label(self):
        ee_point = self.robot.get_ee_position()
        self.ee_point_label.config(text=f'End Point: {round(ee_point[0,0],2)},{round(ee_point[1,0],2)},{round(ee_point[2,0],2)}')

    def loop(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_exit)
        self.root.mainloop()


class Vector:
    def __init__(self, start_point, end_point):
        self.start = np.array([[start_point[0]], [start_point[1]]])
        self.end = np.array([[end_point[0]], [end_point[1]]])

    def magnitude(self):
        return np.sqrt((self.end[0]-self.start[0])**2 + (self.end[1]-self.start[1])**2)


class PlotGenerator:
    def __init__(self, window_size=(12,12)):
        self.window_size = window_size
        self.axes = {}
        self.n_plots = 0
        self.fig = plt.figure(figsize=self.window_size)
        self.fig.canvas.mpl_connect("close_event", self.close)
        self.on_close_callback = None
        self.axes_3D = None

    def generate_axes(self,id,left, bottom, width, height,title,x_label,y_label):
        self.n_plots+=1
        ax: plt.Axes = self.fig.add_axes([left,bottom, width, height])
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        # ax.set_label(label)
        ax.set_title(title)
        self.axes[id]={
            "ax":ax,
            "plots":{}
        }
        return ax
        
    def add_plot_to_axces(self,ax_id,plot_id,data_x,data_y,label):

        logger.debug(data_x,data_y)
        self.axes[ax_id]['plots'][plot_id] = self.get_2D_axes(ax_id).plot(data_x,data_y,label=label)[0]
        return self.axes[ax_id]['plots'][plot_id]

    def get_2D_axes(self,ax_id)->plt.Axes:
        return self.axes[ax_id]['ax']
    
    def get_3D_axes(self)->plt.Axes:
        return self.axes_3D 
    
    def get_plot(self,ax_id,plot_id):
        return self.axes[ax_id]['plots'][plot_id]

    def generate_3D_axes(self,ax_id=None,left=0, bottom=0, width=0, height=2,title='Title',x_label='x',y_label='y',z_label='z'):
        if not ax_id: return self.axes_3D
        self.n_plots+=1
        ax: plt.Axes = self.fig.add_axes([left,bottom, width, height],projection='3d', autoscale_on=False)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.set_zlabel(y_label)
        ax.set_title(title)
        # ax.legend()
        self.axes_3D = ax
        self.axes[ax_id]={
            "ax":ax,
            "plots":{}
        }
        return ax

    def generate_window(self):
        # gs = GridSpec(3, 4, figure=self.fig)

        # # gs.update(wspace=2, hspace=1, left=0, right=1)
        # gs.update(wspace=.5, hspace=.5, left=0.01,
        #           right=0.99, top=0.95, bottom=0.05)
        # gs.tight_layout(figure=self.fig, pad=10, w_pad=5)
        # plt.subplots_adjust()

        # create sub plots as grid


        # For parameter graph visualization
        # self.ax2 = self.fig.add_subplot(gs[0, 2:])
        # self.ax3 = self.fig.add_subplot(gs[1, 2:])
        # self.ax4 = self.fig.add_subplot(gs[2, 2:])
        plt.show()

    def on_close(self,callback):
        self.on_close_callback = callback

    def close(self,args=None):
        plt.close()
        self.on_close_callback()

    
class TrajectoryPlotter:
    def __init__(self, plot_generator:PlotGenerator,start_graph_location:list,graph_gap:float):
        self.plot_generator = plot_generator
        self.start_graph_location =start_graph_location
        self.graph_gap = graph_gap
        self.trajectory = None
        self.trajectory_plot = None
        # self.marker_color = "red"
        self.marker_color = "Lime"
        self.marker_type = "dot"
        self.marker_size = 3
        self.n = 0
        self.theta = np.zeros((200,3))
        self.T = None
        self.ref_2D_plots = []
        self.instant_no = 0

    def set_plot_params(self, marker_size=3, marker_color="red", marker_type="dot"):
        self.marker_size = marker_size
        self.marker_color = marker_color
        self.marker_type = marker_type

    def plot_3D_trajectory(self,trajectory:Trajectory):
        if self.trajectory_plot:
            self.trajectory_plot.remove()

        self.trajectory = trajectory
        # logger.debug("Q :", trajectory.q)
        points_mat = None
        if (self.trajectory.trajectory_method == Trajectory.JOINT_SPACE):
            points_mat = self.trajectory.to_cartesian_space()      
            # logger.debug(points_mat)
        else:
            points_mat = self.trajectory.to_joint_space()
        self.ref_X, self.ref_Y, self.ref_Z = points_mat[:,0],points_mat[:,1],points_mat[:,2]

        self.trajectory_plot = self.plot_generator.get_3D_axes().scatter(
            self.ref_X, self.ref_Y, self.ref_Z, linewidth=3, color=self.marker_color, s=self.marker_size,marker="o")
        
        plt.draw()

    def plot_ref_trajectory(self,trajectory:Trajectory):
        for i in range(self.n):
            ax = self.plot_generator.generate_axes(  id=f"theta{i}",
                                                            title="Ref Position vs Actual Position",
                                                            x_label="Time",
                                                            y_label="Theta"+str(i+1),
                                                            left=self.start_graph_location[0],
                                                            bottom=self.start_graph_location[1]+i*self.start_graph_location[3]+self.graph_gap*i,
                                                            width=self.start_graph_location[2],
                                                            height=self.start_graph_location[3])
            
            X = self.T[0,:].reshape(-1,1)                                            
            Y = trajectory.get_ref_variable_data(i).reshape(-1,1)                                             
            ax.plot(X,Y, label="Planned Theta")
            plt.draw()
            plt.pause(0.0001)
        plt.show()
            
    def plot_act_trajectory(self,theta):
        pass

    def int_act_trajectory(self,theta):
        self.theta[0,:]= theta[0,:]
        for i in range(self.n):
            self.plot_generator.add_plot_to_axces(f"theta{i}",f"theta{i}_p_act",self.T[:,self.instant_no],theta[i,:],"Act Trajectory")
        self.instant_no +=1
        plt.draw()
        plt.pause(0.001)
        plt.show()



    def plot_trajectory(self, trajectory:Trajectory,init_state):
        self.T = trajectory.get_time_stamps()
        # self.ref_theta = trajectory.get_joint_variable_data()
        self.plot_3D_trajectory(trajectory)
        self.plot_ref_trajectory(trajectory)
        self.int_act_trajectory(init_state)
      

    def update_act_trajectory(self,theta,instant):
        self.theta[self.instant_no-1,:]= theta[:,0]
        for i in range(self.n):
            logger.debug("Instant :",instant)
            logger.debug("UPDATE : T",self.T[0,:self.instant_no])
            logger.debug("THETA : Th",self.theta[:self.instant_no,i])
            self.plot_generator.get_plot(f"theta{i}",f"theta{i}_p_act").set_data(self.T[0,:self.instant_no],self.theta[:self.instant_no,i])
        self.instant_no+=1
        plt.draw()
        plt.pause(0.0001)


class RoboPlotter:
    def __init__(self,plot_generator:PlotGenerator,robot_model:RobotModel,location=(0,0,.5, .5),frame_display=True, graph_display=True, FRAME_LINE_WIDTH=1, debug=False):
        self.plot_generator = plot_generator
        self.robot = robot_model
        self.debug = debug
        self.location = location
        self.FRAME_LINE_WIDTH = FRAME_LINE_WIDTH
        self.n = robot_model.n
        self.link_plots = [None]*robot_model.n
        self.frame_plots = np.array(
            [[None]*(robot_model.n+1) for _ in range(3)])
        self.frame_display = frame_display
        self.graph_display = graph_display
        self.ax = self.plot_generator.generate_3D_axes('3Daxes',*location,'MIRS ROBOT')

    def init_graphics(self):
        self.display_links()
        if self.frame_display:
            self.display_frames()

    def animate(self):
        for i in range(self.trajectory.n_steps):
            time_stamp, theta, theta_d, theta_dd = self.trajectory.get_cur_goal()
            # logger.debug(f"T{time_stamp} --> theta : {theta}")
            self.step(theta, theta_d, theta_dd)
            # self.update_graphics(self.robot.links)
            self.time_text.set_text(
                f'time = {time_stamp}s')
            self.trajectory.update_cur_goal()
            plt.pause(self.trajectory.delta_t)

    def start_simulation(self, trajectory, call_back):
        self.trajectory = trajectory
        self.step = call_back
        self.animate()

    def display_links(self):
        self.ax.axes.set_xlim3d(left=-self.robot.model.H_MAX,
                                 right=self.robot.model.H_MAX)
        self.ax.axes.set_ylim3d(
            bottom=-self.robot.model.H_MAX, top=self.robot.model.H_MAX)
        self.ax.axes.set_zlim3d(bottom=0, top=self.robot.model.H_MAX)
        self.ax.grid()
        self.ax.set_title('MIRS')
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.time_text = self.ax.text(
            0.5, 0.5, 0.5, s='', transform=self.ax.transAxes)
        self.trajectory = None
        for i in range(0, self.n):
            link = self.robot.links[i]
            link_colour = self.robot.colors[i]
            X, Y, Z = get_coordinates(link)
            self.link_plots[i], = self.ax.plot(
                X, Y, Z, linewidth=3, color=link_colour)

    def display_frames(self):
        for i in range(0, self.n+1):
            frame = self.robot.transform.get_frame(i)
            for axis in range(3):
                co_ax = frame[:-1, 2*axis:2*(axis+1)].T
                self.frame_plots[axis, i], = self.ax.plot(
                    co_ax[:, 0], co_ax[:, 1], co_ax[:, 2], linewidth=self.FRAME_LINE_WIDTH, color=self.robot.axis_colors[axis])

    def update_graphics(self, time_stamp):
        logger.debug(time_stamp)
        if time_stamp:
            self.time_text.set_text(
                f'time = {time_stamp}s')
        # Update links
        for i in range(0, self.n):
            X, Y, Z = get_coordinates(self.robot.links[i])
            self.link_plots[i].set_data_3d(X, Y, Z)

        if self.frame_display:
            for i in range(0, self.n+1):
                frame = self.robot.transform.get_frame(i)
                for axis in range(3):
                    co_ax = frame[:-1, 2*axis:2*(axis+1)].T
                    self.frame_plots[axis, i].set_data_3d(
                        co_ax[:, 0], co_ax[:, 1], co_ax[:, 2])
        plt.draw()
        plt.pause(Trajectory.DELTA_T)


class SimpleObject:
    SHAPES = ['cube', 'sphere', 'cone', 'cylinder', 'cuboid']

    def __init__(self, name, dim=3, shape='cube', mass=5, length=0.1, width=0.1, height=0.1, gravity=-9.81, coef_res=0.9):
        self.name = name
        self.coef_res = coef_res
        self.id = uuid.uuid1()
        self.shape = shape
        self.length = length
        self.width = width
        self.height = height
        self.origin = np.eye(4)
        self.mass = mass
        self.position = np.zeros((dim, 1))
        self.velocity = np.zeros((dim, 1))
        self.acceleration = np.zeros((dim, 1))
        self.cg = np.array(
            [[self.length / 2], [self.width / 2], [self.height / 2]])
        self.weight = np.array([[0],
                                [0],
                                [mass*gravity]])

    def get_state(self):
        return self.position, self.velocity, self.acceleration

    def set_acceleration(self, acc):
        self.acceleration = acc

    def get_projected_area(self):
        if self.shape == 'cube':
            return self.length*self.length
        elif self.shape == 'sphere':
            return 0.25*np.pi*self.diameter**2

    def set_state(self, position, velocity=[], acceleration=[], time_stamp=0):
        idx = 0 if np.ndim(position) == 1 else [0]

        self.position[:, idx] = position

        if len(velocity) > 0:
            self.velocity[:, idx] = velocity

        if len(acceleration) > 0:
            self.acceleration[:, idx] = acceleration
        # logger.debug(
        #     f"STATE_{obj.name} at {time_stamp}s Position :{obj.position[2,0]} Velocity :{obj.velocity[2,0]} Acceleration : {obj.acceleration[2,0]}")

    def apply_force(self, forces):
        res_F = np.copy(self.weight)

        for force in forces:
            res_F += force

        self.set_acceleration(np.round(res_F/self.mass, 4))


class World:
    def __init__(self, name, dt=0.1, ground=[-np.inf, -np.inf, 0], air=[0, 0, 0], air_density=1.25, out=None):
        self.name = name
        self.__objects = {}
        self.n_ojects = 0
        self.time_stamp = 0
        self.dt = dt
        self.ground = np.array(ground).reshape(-1, 1)
        self.air_density = air_density
        self.air = np.zeros((3, 1))
        self.air[:, 0] = air
        self.air_res = True if len(air) > 0 else False
        # logger.debug(f"Air resistance,Net force,Acceleration,Velocity,Position")

    def write_output(self, F_air, weight, acc, cur_vel, cur_pos):

        logger.debug(
            f"{tuple(F_air.flatten())},{tuple((F_air+weight).flatten())},{tuple(acc.flatten())},{tuple(cur_vel.flatten())},{tuple(cur_pos.flatten())}")

    def get_air_res(self, obj):
        F_air = 0.5*self.air_density*obj.get_projected_area()*(self.air-obj.velocity)**2
        # logger.debug("F_air :", F_air)
        return np.round(F_air, 4)

    def set_objects(self, objects):
        for obj in objects:
            self.__objects[obj.id] = obj
            self.n_ojects += 1

    def get_first_object(self):
        return next(iter(self.get_objects()))

    def get_object(self, id):
        return self.__objects[id]

    def get_objects(self):
        return self.__objects.values()

    def on_object_state_change(self, id, position, *args):
        self.get_object(id).set_state(position, *args)

    def compute_state(self, dt, pre_pos, pre_vel, acc):
        cur_vel = pre_vel + acc * dt
        cur_pos = pre_pos + pre_vel * dt + acc * dt**2
        return cur_pos, cur_vel

    def update(self, time_stamp):
        self.time_stamp = time_stamp

        for obj in self.get_objects():
            F_air = self.get_air_res(obj)
            obj.apply_force(F_air)
            pre_pos, pre_vel, acc = obj.get_state()
            # logger.debug(f"pre pos :{pre_pos}, pre vel :{pre_vel}, acc :{acc}")

            cur_pos, cur_vel = self.compute_state(
                self.dt, pre_pos, pre_vel, acc)

            if (cur_pos < self.ground).any():
                displacements = self.ground-pre_pos
                ax_collision = np.argmin(np.abs(displacements))

                # Compute the velocity before & after collision
                val = pre_vel[ax_collision, 0]**2 + 2 * \
                    acc[ax_collision, 0]*displacements[ax_collision, 0]

                if val > 0:

                    v_b_collision = np.sqrt(val)

                    # Handling the sign of velocity
                    v_b_collision = -1 * v_b_collision if pre_vel[ax_collision, 0] < 0 \
                        else v_b_collision

                    # Direction gets reversed after collision
                    v_a_collision = -obj.coef_res*v_b_collision
                    t_collision = (v_b_collision -
                                   pre_vel[ax_collision, 0])/acc[ax_collision, 0]

                    # Compute new velocity of the object along collision axis after the time stamp
                    p, v = self.compute_state(
                        (self.dt-t_collision), self.ground[ax_collision, 0], v_a_collision, acc[ax_collision, 0])
                    # logger.debug(
                    #     f"Collision at {round(time_stamp-self.dt+t_collision,4)}s Position :{self.ground[ax_collision, 0]} Vecolity: {v_a_collision} Acceleration: {acc[ax_collision, 0]}")

                    cur_pos[ax_collision, 0] = p
                    cur_vel[ax_collision, 0] = v
                else:
                    cur_vel[:, 0] = [0, 0, 0]

            self.write_output(F_air, obj.weight, acc, cur_vel, cur_pos)
            obj.set_state(cur_pos, cur_vel, time_stamp=time_stamp)


class Simulation(TrajectoryPlotter,RoboPlotter):
    def __init__(self, world, robot_model,window_size,robot_plot_location,start_graph_location,graph_gap):
        self.pg = PlotGenerator(window_size)
        self.pg.on_close(self.end_simulation)
        TrajectoryPlotter.__init__(self,self.pg,start_graph_location,graph_gap)
        RoboPlotter.__init__(self,self.pg,robot_model,location=robot_plot_location)
        self.world = world

        # self.create_objects()

    def create_objects(self):
        for obj in self.world.get_objects():
            # Create axis
            axes = [obj.length, obj.width, obj.height]
            data = np.ones(axes, dtype='bool')
            vox = self.ax.voxels(data, facecolors='red')
            self.plots[obj.id] = vox
    
    def init_simulation(self):
        self.init_graphics()
        self.pg.generate_window()

    def end_simulation(self,args=None):
        logger.debug("ending")
    
    def new_state_update(self, time_stamp, theta, theta_d, control):
        # logger.debug("Control : ",control)
        # logger.debug("Theta_d : ",theta_d)
        self.update_act_trajectory(theta,time_stamp)
        self.update_graphics(time_stamp)

    def print_inputs(*args):
        logger.debug(args)


class Simulation2():
    GRAPHIC_WINDOW_SIZE = (14, 7)

    def __init__(self, world, step_time=0.1, plot=False):
        self.plots = {}
        self.step_time = step_time
        self.world = world
        self.P = []
        self.t = []
        self.V = []

        if plot:
            self.fig = plt.figure(figsize=(5, 5))
            self.ax = self.fig.add_subplot(111, projection='3d')
            # viewrange for z-axis should be [-4,4]
            self.ax.set_zlim3d(-100, 500)
            self.ax.set_ylim3d(-100, 500)
            self.ax.set_xlim3d(-100, 500)
            # self.ax = self.fig.add_subplot(111)
            self.create_objects()
        self.fig.canvas.mpl_connect('close_event', exit)

    def exit(self):
        exit(0)

    def update(self, t):
        for obj in self.world.get_objects():
            self.P.append(obj.position[2, 0])
            self.t.append(t)
            self.V.append(obj.velocity[2, 0])
            # self.plots[obj.id].remove()
            logger.debug(*obj.position)
            self.plots[obj.id].set_data_3d(*obj.position)
            # self.plots[obj.id] = self.ax.plot(self.t, self.P, color='red')[0]
        plt.draw()
        plt.pause(self.step_time)

    def create_objects(self):

        for obj in self.world.get_objects():
            # self.plots[obj.id] = self.ax.plot(self.t, self.P, color='red')[0]
            self.plots[obj.id] = self.ax.plot(
                *obj.position, marker='o', markersize=20, color='red')[0]
            # Create axis
            # axes = [obj.length, obj.width, obj.height]
            # data = np.ones(axes, dtype=bool)
            # vox = self.ax.voxels(data, facecolors='red')
            # self.plots[obj.id] = vox

    def loop(self, T=30, infinity=False):
        t = 0

        def check_condition():
            if infinity:
                return True
            return t < T

        while check_condition():
            t = round(t + self.step_time, 4)
            # try:
            #     self.world.update(t)
            #     time.sleep(self.step_time)
            #     t = t + self.step_time
            # except Exception as e:
            #     logger.debug(e)
            #     break
            self.world.update(t)
            self.update(t)
            if (self.world.get_first_object().velocity[2, 0]) == 0:
                break


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

    def __init__(self, robot_model, transform:Transform, delta_t=0.1, precision=4, debug=False, simp_sim=False):
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
        self.trajec_gen = TrajectoryGenerator(self.kinematics,
                                                 self.dynamics,
                                                 self.delta_t,
                                                 precision=self.precision,
                                                 trajectory_type=Trajectory.TRAJECTORY_CUBIC)
        self.ai = AI()

        self.theta_hist = []
        self.theta_d_hist= []
        self.theta_dd_hist = []
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
        logger.debug("Exiting from the program...")
        self.on_exit_callback()
        exit(0)

    def move_to_goal(self, goal_point, T, simple_sim=False):
        start_orientation, start_point = self.transform.get_current_pose()

        # ons = np.ones((self.n, 1))
        ons = np.ones((6, 1))

        status, trajectory = self.trajec_gen.generate_trajectory(
            start_point, goal_point, 0, T, 0*ons, 0*ons, 0*ons)
        

        if (not status):
            # logger.debug(trajectory)
            # logger.debug(trajectory)
            return 

        self.on_new_trajectory_callback(trajectory,self.get_state())
    
        trajectory.init_goal()

        if self.simp_sim:
            self.on_simp_sim_callback(trajectory, self.move_joint)
        else:
            self.controller.execute(trajectory)
            

    def init_history(self,trajectory):
        self.theta

    def add_noise(self, th, noise_mean=0, noise_var=0.0006):
        return np.round(th + np.random.normal(noise_mean, noise_var, size=(self.n, 1)), self.precision)

    def step(self, u, time_stamp=None, add_noise=True,update_anim=True):
        print("Stepping :",time_stamp)
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
            logger.debug(f"T : {u}")
            logger.debug(f"theta :{theta}")
            logger.debug(f"theta_d :{theta_d}")
            logger.debug(f"theta_dd :{self.theta_dd}")
        
        if(update_anim):
            self.move_joint(theta, theta_d, self.theta_dd)
        else:
            self.theta[:, :] = theta
            self.theta_d[:, :] = theta_d
            self.theta_dd[:, :] = self.theta_dd

        self.transform.update(theta)

    def execute_program(self, file_name="program.txt"):
        actions = ["PICK", "PLACE"]
        movements = ["MOVE_DOWN", "MOVE_UP", "MOVE_LEFT",
                     "MOVE_RIGHT", "MOVE_FRONT", "MOVE_BACK", "MOVE"]

        # logger.debug("File path :", file_name)
        command_list = []
        args_list = []

        with open(file_name, 'r') as f:
            for line in f.readlines():
                res = line.split()
                command_list.append(res[0])
                args_list.append(res[1:])

        for i in range(len(command_list)):
            logger.debug(f"Command :{command_list[i]} Args :{args_list[i]}")
            command = command_list[i]
            args = args_list[i]

            if (command in actions):
                # position = self.ai.get_object_position(args[0])
                position = np.array(args[1:4], dtype=np.float16).reshape(-1, 1)
                logger.debug(f"Action :{str.title(command)}ing object...")
                self.move_to_goal(position, int(args[-1]))
            else:
                logger.debug(f"Movement :{str.title(command)}")
                displacement_vec = self.get_disp_vector(
                    command, float(args[0]))
                logger.debug("Disp vec :", displacement_vec)
                logger.debug("Cur vec :", self.get_ee_position())
                self.move_to_goal(self.get_ee_position() +
                                  displacement_vec, float(args[1]))
            time.sleep(0.1)

    def get_state(self):
        return self.theta[:,:]
    
    def move_joint(self, theta, theta_d=np.ones((3, 1)), theta_dd=np.ones((3, 1))):
        if self.debug:
            logger.debug("Moving Theta : ", theta, "\n\n")

        self.theta[:, :] = theta
        self.theta_d[:, :] = theta_d
        self.theta_dd[:, :] = theta_dd

        self.transform.update(theta)
        self.update_link_pos()

        if (self.controller.controller_running):
            self.on_joint_move_callback(self.time_stamp,self.theta,self.theta_d, self.u)
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

    def get_dof(self):
        return self.n
    

if __name__ == "__main__":
    cm = 1/2.54
    robot_model = RobotModel(n=N, unit=UNIT)
    transform = Transform(robot_model=robot_model)
    robot = Robot(robot_model=robot_model, transform=transform, simp_sim=False)
    console = Console(robot)
    world = World("mirs world")
    # obj = SimpleObject('box')
    # world.set_objects([obj])
    simulation = Simulation(world, 
                            robot_model=robot,
                            window_size=(25*cm,15*cm),
                            robot_plot_location=[0.05,0.05,.45,0.9],
                            start_graph_location=[0.6,0.05,0.35,(1-0.05*(N+1))/N],  # left, bottom, width, height  fractions of figure size
                            graph_gap=0.01)

    robot.on_exit(simulation.end_simulation)
    robot.on_simp_sim(simulation.start_simulation)
    robot.on_joint_move(simulation.new_state_update)
    robot.on_new_trajectory(simulation.plot_trajectory)
    console.on_new_program(robot.execute_program)
    console.on_state_change(robot.get_ee_position)
    console.on_manual_control(robot.move_joint)
    console.on_goal(robot.move_to_goal)
    console.on_exit(robot.exit)


    simulation.init_simulation()
    console.loop()

