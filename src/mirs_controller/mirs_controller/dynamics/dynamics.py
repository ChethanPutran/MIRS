import math
import numpy as np
from sympy import *
import inspect
from sympy.utilities.lambdify import implemented_function
from .C import C
from .G import G
from .M import M


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
        print(self.M.inv())
        return
        q_dd = self.M.inv() @ (self.tau - self.C - self.G)
        q_dd_func = lambdify((self.q, self.q_d, self.tau),
                             q_dd, modules='numpy')
        return q_dd_func


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

            print(self.transform.t(i+1))
            print(self.F[:, [i]])
            self.n_tau[:, [i]] = self.transform.R(i+1) @ self.n_tau[:, [i+1]] +\
                np.cross(self.transform.t(i+1), self.F[:, [i]], axis=0)

            # Calculate joint resisting torque (dot product between torque vector and joint axis)
            self.tau[i, 0] = self.n_tau[:, [i]].T @ self.Zi
        return self.F, self.tau


if __name__ == "__main__":
    # robot = Robot()
    # deg = DynamicsEquationGenerator(robot)
    # deg.generate_dynamic_param_functions()
    from ..mirs import Transform, RobotModel

    robot = RobotModel()
    tf = Transform(robot)
    dn = Dynamics(tf)

    # t = dn.forward([np.pi/2, np.pi/4, np.pi/6], [0.1, 0.1, 0.1], [1, 1, 1])
    # print(t)
    # print(dn.inverse(list(t.T), [np.pi/2, np.pi/4, np.pi/6], [0.1, 0.1, 0.1]))
