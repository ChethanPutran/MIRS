import numpy as np
# import roboticstoolbox as rtb
# from  mirs_controller.transformations.matrices import Matrices
import sympy as sym
from sympy import Matrix,Symbol,zeros,eye,Array
from urdf_parser_py.urdf import URDF
from ..kinematics.kinematics import Kinematics
import xacro
# import roboticstoolbox as rtb
# from  mirs_controller.transformations.matrices import Matrices




from sympy import symbols, Matrix, simplify, diff
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame

def calculate_dynamic_equation(transforms, inertias, masses, lengths):
    # Symbols for joint variables
    q1, q2, q3, q4, q5, q6 = dynamicsymbols('q1:7')
    q = Matrix([q1, q2, q3, q4, q5, q6])

    # Define reference frame
    N = ReferenceFrame('N')

    # Calculate velocities and accelerations
    q_dot = Matrix([diff(qi, symbols('t')) for qi in q])
    q_ddot = Matrix([diff(qi, symbols('t', 2)) for qi in q])

    # Calculate kinetic energy
    T = 0
    for i, transform in enumerate(transforms):
        R_i = transform[:3, :3]
        omega_i = R_i.T * N.ang_vel_in(N)
        v_i = R_i.T * N.vel(N)
        T_i = 0.5 * masses[i] * v_i.dot(v_i) + 0.5 * omega_i.dot(inertias[i] * omega_i)
        T += simplify(T_i)

    # Calculate potential energy
    V = 0
    for i in range(len(masses)):
        V += masses[i] * lengths[i] * N.down().dot(transforms[i][:3, 2])

    # Calculate Lagrangian
    L = T - V

    # Calculate generalized forces
    tau = Matrix([dynamicsymbols('tau%i' % (i + 1)) for i in range(len(q))])

    # Calculate Lagrange's equation
    lagrange_eqs = Matrix([simplify(diff(diff(L, q_dot[i]), symbols('t'))) - diff(L, q[i]) + tau[i] for i in range(len(q))])

    return lagrange_eqs


import numpy as np

def calculate_dynamic_equation(theta, d, a, alpha, m, r, I, g):
    # theta: joint angles
    # d: link offsets
    # a: link lengths
    # alpha: link twists
    # m: link masses
    # r: link center of masses
    # I: link inertias
    # g: gravity vector

    num_joints = len(theta)

    # Initialize variables
    joint_angles = np.asarray(theta)
    link_offsets = np.asarray(d)
    link_lengths = np.asarray(a)
    link_twists = np.asarray(alpha)
    link_masses = np.asarray(m)
    link_centers_of_masses = np.asarray(r)
    link_inertias = np.asarray(I)
    gravity = np.asarray(g)

    # Transformation matrices
    transforms = []
    for i in range(num_joints):
        theta_i = joint_angles[i]
        d_i = link_offsets[i]
        a_i = link_lengths[i]
        alpha_i = link_twists[i]

        transform = np.array([[np.cos(theta_i), -np.sin(theta_i) * np.cos(alpha_i), np.sin(theta_i) * np.sin(alpha_i), a_i * np.cos(theta_i)],
                              [np.sin(theta_i), np.cos(theta_i) * np.cos(alpha_i), -np.cos(theta_i) * np.sin(alpha_i), a_i * np.sin(theta_i)],
                              [0, np.sin(alpha_i), np.cos(alpha_i), d_i],
                              [0, 0, 0, 1]])
        transforms.append(transform)

    # Calculate the Lagrangian equations of motion
    q = joint_angles
    q_dot = np.zeros(num_joints)
    q_double_dot = np.zeros(num_joints)

    # Calculate the kinetic energy
    kinetic_energy = 0
    for i in range(num_joints):
        transform_i = np.eye(4)
        for j in range(i+1):
            transform_i = np.dot(transform_i, transforms[j])

        R_i = transform_i[:3, :3]
        p_i = transform_i[:3, 3]

        # Calculate the angular velocity
        omega_i = np.dot(np.linalg.inv(R_i), q_dot[i] * np.array([0, 0, link_twists[i]]))

        # Calculate the linear velocity
        v_i = np.cross(omega_i, p_i)

        # Calculate the kinetic energy of the link
        kinetic_energy += 0.5 * link_masses[i] * np.dot(v_i, v_i) + 0.5 * np.dot(np.dot(omega_i, link_inertias[i]), omega_i)

    # Calculate the potential energy
    potential_energy = 0
    for i in range(num_joints):
        transform_i = np.eye(4)
        for j in range(i+1):
            transform_i = np.dot(transform_i, transforms[j])

        p_i = transform_i[:3, 3]  # Position vector of ith link w.r.to base frame

        # p_ci = p_i * (R_i*p_i_ci)

        # Calculate the potential energy of the link
        potential_energy += link_masses[i] * np.dot(gravity, p_i)  # m_i * g.T *p_0_ci

    # Calculate the Lagrangian
    lagrangian = kinetic_energy - potential_energy

    # Calculate the Lagrange equation
    for i in range(num_joints):
        # Calculate the partial derivatives of the Lagrangian with respect to the generalized coordinates
        partial_lagrangian = np.diff([lagrangian], [q[i]])

        # Calculate the derivative of the partial


import sympy as sp

def calculate_dynamic_equation(theta, d, a, alpha, m, r, I):
    # theta: joint angles (symbolic)
    # d: link offsets
    # a: link lengths
    # alpha: link twists
    # m: link masses
    # r: link center of masses
    # I: link inertias

    num_joints = len(theta)

    # Initialize symbolic variables
    q = sp.Matrix(theta) # Joint angles
    q_dot = sp.Matrix([sp.Symbol('q{}_dot'.format(i+1)) for i in range(num_joints)]) # Joint velocities
    q_double_dot = sp.Matrix([sp.Symbol('q{}_ddot'.format(i+1)) for i in range(num_joints)]) # Joint accelerations
    g = sp.symbols('g') # Gravity

    # Transformation matrices
    T = []
    for i in range(num_joints):
        theta_i = q[i]
        d_i = d[i]
        a_i = a[i]
        alpha_i = alpha[i]

        T_i = sp.Matrix([[sp.cos(theta_i), -sp.sin(theta_i) * sp.cos(alpha_i), sp.sin(theta_i) * sp.sin(alpha_i), a_i * sp.cos(theta_i)],
                         [sp.sin(theta_i), sp.cos(theta_i) * sp.cos(alpha_i), -sp.cos(theta_i) * sp.sin(alpha_i), a_i * sp.sin(theta_i)],
                         [0, sp.sin(alpha_i), sp.cos(alpha_i), d_i],
                         [0, 0, 0, 1]])
        T.append(T_i)

    # Joint positions and velocities
    p = []
    v = []
    for i in range(num_joints):
        T_i = sp.eye(4)
        for j in range(i+1):
            T_i = T_i * T[j]

        p_i = T_i[:3, 3]
        v_i = sp.diff(p_i, q[0])*q_dot[0]
        for j in range(1, i+1):
            v_i += sp.diff(p_i, q[j])*q_dot[j]

        p.append(p_i)
        v.append(v_i)

    # Kinetic energy
    kinetic_energy = 0
    for i in range(num_joints):
        R_i = T[i][:3, :3]
        v_i = v[i]
        I_i = sp.Matrix(I[i])

        kinetic_energy += 0.5 * m[i] * sp.dot(v_i, v_i) + 0.5 * sp.dot(sp.dot(v_i, R_i), I_i * sp.transpose(sp.dot(R_i, v_i)))

    # Potential energy
    potential_energy = 0
    for i in range(num_joints):
        p_i = p[i]
        potential_energy -= m[i] * g * p_i[2]

    # Lagrangian
    lagrangian = kinetic_energy - potential_energy

    # Lagrange's equations
    lagrange_equations = []
    for i in range(num_joints):
        dL_dq_dot = sp.diff(lagrangian, q_dot[i])
        ddL_dq_dot_dt = sp.diff(dL_dq_dot, q[i]) * q_dot[i] + dL_dq_dot.diff(q_dot[i])


import numpy as np

def calculate_dynamic_equation(transforms, inertias, masses, lengths, joint_angles, joint_velocities, joint_accelerations):
    # transforms: list of transformation matrices for each joint
    # inertias: list of inertias for each link
    # masses: list of masses for each link
    # lengths: list of link lengths
    # joint_angles: array of joint angles
    # joint_velocities: array of joint velocities
    # joint_accelerations: array of joint accelerations

    num_joints = len(transforms)

    # Initialize variables
    dynamic_matrix = np.zeros((num_joints, num_joints))
    dynamic_vector = np.zeros(num_joints)

    for i in range(num_joints):
        transform_i = transforms[i]
        R_i = transform_i[:3, :3]
        p_i = transform_i[:3, 3]

        # Calculate the inertia tensor in the world frame
        inertia_world = np.dot(R_i, np.dot(inertias[i], R_i.T))

        # Calculate the centrifugal and Coriolis forces
        force_centrifugal_coriolis = np.zeros(3)
        torque_centrifugal_coriolis = np.zeros(3)
        for j in range(i):
            transform_j = transforms[j]
            R_j = transform_j[:3, :3]
            p_j = transform_j[:3, 3]

            omega_j = np.dot(R_j.T, np.dot(R_i, joint_velocities[j]))
            v_j = np.cross(omega_j, np.dot(R_i, p_i - p_j))
            force_centrifugal_coriolis += masses[j] * v_j
            torque_centrifugal_coriolis += np.dot(inertia_world, omega_j)

        # Calculate the gravitational forces
        gravity_force = np.zeros(3)
        if i == 0:
            gravity_force = np.array([0, 0, -9.81 * masses[i]])
        else:
            gravity_force = np.dot(R_i, np.array([0, 0, -9.81 * masses[i]]))

        # Calculate the joint acceleration
        accel_i = joint_accelerations[i]

        # Calculate the joint torque
        torque_i = np.dot(inertia_world, accel_i) + np.cross(joint_velocities[i], np.dot(inertia_world, joint_velocities[i])) + np.dot(inertia_world, torque_centrifugal_coriolis) + np.dot(R_i, force_centrifugal_coriolis) + gravity_force

        # Update the dynamic equation matrix and vector
        dynamic_matrix[i, :] = torque_i
        dynamic_vector[i] = -np.dot(torque_i, joint_velocities[i])

    return dynamic_matrix, dynamic_vector



import numpy as np

def calculate_forward_dynamics(theta, d, a, alpha, m, r, I, g):
    # theta: joint angles
    # d: link offsets
    # a: link lengths
    # alpha: link twists
    # m: link masses
    # r: link center of masses
    # I: link inertias
    # g: gravity vector

    num_joints = len(theta)

    # Initialize variables
    joint_angles = np.asarray(theta)
    link_offsets = np.asarray(d)
    link_lengths = np.asarray(a)
    link_twists = np.asarray(alpha)
    link_masses = np.asarray(m)
    link_centers_of_masses = np.asarray(r)
    link_inertias = np.asarray(I)
    gravity = np.asarray(g)

    # Transformation matrices
    transforms = []
    for i in range(num_joints):
        theta_i = joint_angles[i]
        d_i = link_offsets[i]
        a_i = link_lengths[i]
        alpha_i = link_twists[i]

        transform = np.array([[np.cos(theta_i), -np.sin(theta_i) * np.cos(alpha_i), np.sin(theta_i) * np.sin(alpha_i), a_i * np.cos(theta_i)],
                              [np.sin(theta_i), np.cos(theta_i) * np.cos(alpha_i), -np.cos(theta_i) * np.sin(alpha_i), a_i * np.sin(theta_i)],
                              [0, np.sin(alpha_i), np.cos(alpha_i), d_i],
                              [0, 0, 0, 1]])
        transforms.append(transform)

    # Forward dynamics calculation
    joint_accelerations = []
    forces = []
    torques = []

    for i in range(num_joints):
        transform_i = np.eye(4)
        for j in range(i+1):
            transform_i = np.dot(transform_i, transforms[j])

        R_i = transform_i[:3, :3]
        p_i = transform_i[:3, 3]

        # Calculate the acceleration due to gravity
        force_gravity = link_masses[i] * gravity
        force_gravity_transformed = np.dot(R_i, force_gravity)

        # Calculate the centrifugal and Coriolis forces
        force_centrifugal_coriolis = np.zeros(3)
        torque_centrifugal_coriolis = np.zeros(3)

        for j in range(i):
            omega_j = np.cross(transforms[j][:3, 2], joint_velocities[j])
            v_j = np.cross(omega_j, transforms[j][:3, 3])
            force_centrifugal_coriolis += link_masses[j] * v_j
            torque_centrifugal_coriolis += np.dot(link_inertias[j], omega_j)

        # Calculate the joint acceleration
        inertia_i = np.dot(R_i, np.dot(link_inertias[i], R_i.T))
        alpha_i = np.dot(np.linalg.inv(inertia_i), torque_centrifugal_coriolis)
        accel_i = np.dot(np.linalg.inv(inertia_i), force_gravity_transformed - np.cross(joint_velocities[i], np.dot(inertia_i, joint_velocities[i])) - np.cross(joint_velocities[i], np.dot(inertia_i, force_centrifugal_coriolis)))

        joint_accelerations.append(accel_i)
        forces.append(force_gravity_transformed)
        torques.append(torque_centrifugal_coriolis)

    return joint_accelerations, forces,


class Dynamics:
    def init(self, urdf_file=r"C:\Chethan\Mechanical\projects\Major_Project\software\MIRS\mirs\src\mirs_description\src\description\robot.urdf", n_joints=6, gx=0, gy=0, gz=-9.81):
        self.xacro_file = urdf_file
        self.n = n_joints

        # # Process xacro file
        xml_string = xacro.process_file(self.xacro_file).toxml()
        self.robot = URDF()
        self.robot = self.robot.from_xml_string(xml_string)
        self.robot.

        # Check if robot model is valid
        # self.robot.check_valid()

        # # self.matrices = Matrices()
        self._I = {
            "I1": [[0]*3]*3,
            "I2": [[0]*3]*3,
            "I3": [[0]*3]*3,
            "I4": [[0]*3]*3,
            "I5": [[0]*3]*3,
            "I6": [[0]*3]*3,
        }
        self._m = {
            "m1": 0,
            "m2": 0,
            "m3": 0,
            "m4": 0,
            "m5": 0,
            "m6": 0,
        }
        self._Cm = {
            "cm1": [0]*3,
            "cm2": [0]*3,
            "cm3": [0]*3,
            "cm4": [0]*3,
            "cm5": [0]*3,
            "cm6": [0]*3,
        }

        m_keys = list(self._m.keys())
        i_keys = list(self._I.keys())
        c_keys = list(self._Cm.keys())

        for i, link in enumerate(list(self.robot.link_map.keys())[1:self.n+1]):
            self._m[m_keys[i]] = self.robot.link_map[link].inertial.mass
            inertia = self.robot.link_map[link].inertial.inertia
            self._I[i_keys[i]][:] = [[inertia.ixx, inertia.ixy, inertia.ixz],
                                      [inertia.ixy, inertia.iyy, inertia.iyz],
                                      [inertia.ixz, inertia.iyz, inertia.izz]]
            self._Cm[c_keys[i]][:] = self.robot.link_map[link].inertial.origin.xyz

        # # self.read_inertia_mats(urdf_file)
        # self.n_joints = n_joints
        # self.__D_i = np.zeros((self.n_joints,1))
        # self.__D_ij = np.zeros(self.n_joints)
        # self.__D_ijk = np.zeros((self.n_joints,1))
        # self.__U_ij= np.zeros((self.n_joints,1))
        # self.__U_ijk= np.zeros((self.n_joints,1))
        # self.__J_i =  np.zeros((n_joints,4,4))
        # self.__g_T = [gx,gy,gz,0]
        # self.__T_i = np.zeros((self.n_joints,1))

        # # puma = rtb.models.DH.Puma560()
        # # tau = puma.rne(puma.qn, np.zeros((6,)), np.zeros((6,)))
        # # print(tau)

    def Compute_P_i(self, r_i, i):
        """ 
            -> Returns position of 'i' th link w.r.to base frame\n
            -> r_i - Location of any point on 'i' th link
                w.r.to reference frame in the 'i' th link
        """
        return np.dot(self.matrices.HomogenousTM(0, i), r_i)

    def Compute_U_ij(self, i, j):
        """
            Returns del(0_T_i)/del(q_j)
        """
        assert j <= i, "j should be less than or equal to i"

        del_T_by_del_q = np.dot(self.matrices.HomogenousTM(0, j),
                                self.D(),
                                self.matrices.HomogenousTM(j, i))

        return del_T_by_del_q

    def Compute_U_ij(self, i, j, k=None):
        """
            Returns del(del(0_T_i)/del(q_j))/del(q_k)
        """
        assert j <= i and k <= i, "j & k should be less than or equal to i"

        p = j  # min
        q = k  # max

        if (k and j > k):
            # If k is less than l
            p = k
            q = j

        del_T_by_del_q = np.dot(self.matrices.HomogenousTM(0, p),
                                self.D(),
                                self.matrices.HomogenousTM(p, q),
                                self.D(),
                                self.matrices.HomogenousTM(q, i))

        return del_T_by_del_q

    def Compute_J_i(self, i):
        """
            Return inertia tensor of ith link
            integ(r_i*r_i_T*dm_i)

        """
        Ixx = self._I[f"I{i}"][0]
        Iyy = self._I[f"I{i}"][0]
        Izz = self._I[f"I{i}"][0]
        Ixy = self._I[f"I{i}"][0]
        Iyz = self._I[f"I{i}"][0]
        Ixz = self._I[f"I{i}"][0]
        m = self._m[f"m{i}"]
        x_bar, y_bar, z_bar = self.__cg[f"cg{i}"]

        J_i = [[0.5*(-Ixx+Iyy+Izz),     Ixy,             Ixz,        m*x_bar],
               [Ixy,          0.5*(Ixx-Iyy+Izz),      Iyz,        m*y_bar],
               [Ixz,                Iyz,       0.5*(Ixx+Iyy-Izz), m*z_bar],
               [m*x_bar,           m*y_bar,          m*z_bar,         1]
               ]

        return J_i

    def Compute_D_i(self, i):
        pass

    def Compute_D_ij(self, i, j):
        """
         It calculates the inertia term

        """
        sum_ = 0

        for p in range(max(i, j), self.n_joints+1):

            # U_pj
            U = self.U_ijk(p, j)

            # Jp
            J = self.J_i(p)

            # U_pi_T
            U_T = self.U_ijk(p, i).T

            sum_ += np.trace(np.dot(U, J, U_T))
        return sum_

    def Compute_D_ijk(self, i, j, k):
        """
         It calculates the centrifugal & coriolis term

        """
        for p in range(max(i, j, k), self.n_joints+1):
            self.__D_ijk[i][p] += 0

    def T_i(self, i, q, q_dot, q_dotdot):
        """
            Calculates the 'i' th joint torque required

        """

        D_i = []
        D_ij = []
        D_ijk = []
        # Inertia matrix
        M = [self.D_ij(i, j) for j in range(1, self.n_joints+1)]

        A = np.dot(D_ij[i], q_dotdot)
        B = np.dot((np.dot(D_ijk, q_dot).T), q_dot)
        C = D_i[i]

        T = A + B + C

    def del_T_by_del_q_i(self, i):
        o_T_i_1 = self.matrices.HomogenousTM(0, i-1)
        i_1_T_n = self.matrices.HomogenousTM(i-1, self.n_joints)
        D = self.D(type='revolute')

        del_T_by_del_q = np.dot(o_T_i_1, D, i_1_T_n)

        # or
        np.dot(self.matrices.HomogenousTM(0, i), self.T_D(
            self.matrices.HomogenousTM(i-1, i)))

        return del_T_by_del_q

    def del_0_T_j_by_del_t(self, j, theta_dot):
        # delT/delq = J
        J = []

        del_T_by_del_t = np.dot(J.T, theta_dot)

        return del_T_by_del_t

    """
    Returns d((i-1)_T_i)/d(q_i) : D

    """

    def T_D(self, T, delta):
        """
        Returns the differential transformation of 'i' th frame
        w.r.to 'i-1' th frame due to change 'i' ith joint variable

        """

        n = T[:, 0]
        o = T[:, 0]
        a = T[:, 0]
        p = T[:, 0]

        T_del_x = np.dot(delta, n)
        T_del_y = np.dot(delta, o)
        T_del_z = np.dot(delta, a)

        # t_d_y = n_x * p_y + n_y * p_x
        T_d_x = -n[0]*p[1] + n[1]*p[0]

        # t_d_y = o_x * p_y + o_y * p_x
        T_d_y = -o[0]*p[1] + o[1]*p[0]

        # t_d_z = a_x * p_y + a_y * p_x
        T_d_z = -a[0]*p[1] + a[1]*p[0]

        t_D = [[0, -T_del_z,  T_del_y, T_d_x],
               [T_del_z,     0, -T_del_x, T_d_y],
               [-T_del_y,  T_del_x,     0, T_d_z],
               [0,     0,     0,   0],
               ]

        # or
        # D_T = np.dot(np.linalg.inv(T),self.D(),T)

        return t_D

    def D(self, type='revolute'):
        """
        Returns the differential transformation of point in 'i' th frame
        w.r.to 'i' th frame due to change 'i' ith joint variable

        """

        if (type == "revolute"):
            # Rotation vector
            K = [0, 0, 1]
            D = [0, 0, 0]

        elif (type == "prismatic"):
            # Rotation vector
            K = [0, 0, 0]
            D = [0, 0, 1]

        delta_theta = 1
        Kx, Ky, Kz = K
        delta_x, delta_y, delta_z = D

        delta_theta_x = Kx*delta_theta
        delta_theta_y = Ky*delta_theta
        delta_theta_z = Kz*delta_theta

        # For small changes
        D = [
            [0, -delta_theta_z,  delta_theta_y, delta_x],
            [delta_theta_z,        0, -delta_theta_x, delta_y],
            [-delta_theta_y, -delta_theta_x,       0, delta_z],
            [0,        0,       0,    0],
        ]

        return D

    def D_dot(self, theta, theta_dot):
        pass

    def M(self, theta, theta_dotdot):
        sum = 0
        for j in range(1, self.n_joints+1):
            sum += self.Dij(j)*theta_dotdot[j]

    def C(self, theta, theta_dot):
        pass

    def G(self, theta):
        pass

    def F(self, theta, theta_dot):
        pass

    def compute(self, theta, theta_dot, theta_dotdot):
        # Dynamic equation
        # tau = M*q_dot_dot + C*q_dot + G
        tau = self.M(theta)*theta_dotdot + self.C(theta,
                                                  theta_dot)*theta_dot + self.G(theta)
        return tau

    def inertial_params(self, symbol=True):
        ''' 
        Returns a struct with inertial parameters of 6 DOF MIR obtained from MIR's xacro file 
        '''

        def get_inertial_tensor_symbol(i):
            Ixx =Symbol(f'I{i}xx')
            Ixy =Symbol(f'I{i}xy')
            Ixz =Symbol(f'I{i}xz')
            Iyy =Symbol(f'I{i}yy')
            Iyz =Symbol(f'I{i}yz')
            Izz =Symbol(f'I{i}zz')

            return sym.Matrix([[Ixx, Ixy, Ixz],
                               [Ixy, Iyy, Iyz],
                               [Ixz, Iyz, Izz]])

        def get_cm_symbol(i):
            cx =Symbol(f'c{i}x')
            cy =Symbol(f'c{i}y')
            cz =Symbol(f'c{i}z')

            return sym.Array([cx, cy, cz])

        # Extract information from node
        # If enable == TRUE then numeric values are returned, else symbolic values are returned

        params = {
            'm': [0]*self.n,
            'I': zeros(3,3*self.n),
            'Cm': [0]*3*self.n,

        }
        if not symbol:
            params['m'][:] = [self._m[key] for key in self._m]
            params['I'][:, 0:3]= self._I['I1']
            params['I'][:, 3:6] = self._I['I2']
            params['I'][:, 6:9] = self._I['I3']
            params['I'][:, 9:12] = self._I['I4']
            params['I'][:, 12:15] = self._I['I5']
            params['I'][:, 15:18] = self._I['I6']

            params['Cm'][0:3] = self._Cm['cm1']
            params['Cm'][3:6] = self._Cm['cm2']
            params['Cm'][6:9] = self._Cm['cm3']
            params['Cm'][9:12] = self._Cm['cm4']
            params['Cm'][12:15] = self._Cm['cm5']
            params['Cm'][15:18] = self._Cm['cm6']

        else:
            m1 =Symbol('m1')
            m2 =Symbol('m1')
            m3 =Symbol('m3')
            m4 =Symbol('m4')
            m5 =Symbol('m5')
            m6 =Symbol('m6')

            params['m'] = sym.Matrix([m1, m2,m3,m4,m5,m6])  # 6x1

            params['I'][:, 0:3] = get_inertial_tensor_symbol(1)
            params['I'][:, 3:6] = get_inertial_tensor_symbol(2)
            params['I'][:, 6:9] = get_inertial_tensor_symbol(3)
            params['I'][:, 9:12] = get_inertial_tensor_symbol(4)
            params['I'][:, 12:15] = get_inertial_tensor_symbol(5)
            params['I'][:, 15:18] = get_inertial_tensor_symbol(6)

            params['Cm'][0:3] = get_cm_symbol(1)
            params['Cm'][3:6] = get_cm_symbol(2)
            params['Cm'][6:9] = get_cm_symbol(3)
            params['Cm'][9:12] = get_cm_symbol(4)
            params['Cm'][12:15] = get_cm_symbol(5)
            params['Cm'][15:18] = get_cm_symbol(6)

        return params

    def jacobian(self, F, v):
        F.jacobian(v)

    def make_homogenous(self, mat):
        return sym.Matrix()

    def compute_dynamics(self,kom:Kinematics):
        # Inverse dynamics analysis of 6 DOF MIR

        q1 =Symbol('q1')
        q2 =Symbol('q2')
        q3 =Symbol('q3')
        q4 =Symbol('q4')
        q5 =Symbol('q5')
        q6 =Symbol('q6')
        dq1 =Symbol('dq1')
        dq2 =Symbol('dq2')
        dq3 =Symbol('dq3')
        dq4 =Symbol('dq4')
        dq5 =Symbol('dq5')
        dq6 =Symbol('dq6')
        ddq1 =Symbol('ddq1')
        ddq2 =Symbol('ddq2')
        ddq3 =Symbol('ddq3')
        ddq4 =Symbol('ddq4')
        ddq5 =Symbol('ddq5')
        ddq6 =Symbol('ddq6')

        q = Matrix([q1, q2, q3, q4, q5, q6]) # 6x1

        dq = Matrix([dq1,dq2,dq3,dq4,dq5,dq6]) # 6x1

        ddq = Matrix([ddq1, ddq2,ddq3, ddq4, ddq5,ddq6])  # 6x1

        # Inertial parameters (Run inertial_params(False) to obtain numeric values or inertial_params(True) to obtain symbolic values

        params = self.inertial_params(symbol=False)

        # Mass of links
        m = params['m']

        # Inertia tensor of links
        I1 = params['I'][:, 0:3]
        I2 = params['I'][:, 3:6]
        I3 = params['I'][:, 6:9]
        I4 = params['I'][:, 9:12]
        I5 = params['I'][:, 12:15]
        I6 = params['I'][:, 15:18]

        # Center of mass (COM) of links
        com1 = params['Cm'][0:3]
        com2 = params['Cm'][3:6]
        com3 = params['Cm'][6:9]
        com4 = params['Cm'][9:12]
        com5 = params['Cm'][12:15]
        com6 = params['Cm'][15:18]

        
        z0, z1, z2, z3, z4, z5 = kom.forward_differential_kinematics()
        T1_0, T2_0, T3_0, T4_0, T5_0, T6_0 = kom.get_diff_transforms()

        # Compute inertia matrix D

        k = Matrix([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0]])
        
        print("Jacob started")

        # 4x4 4x4 4x1
        r0_c1 = k * T1_0 * Matrix([*com1, 1])  # 4x1
        Jl_1 = r0_c1.jacobian(q)
        Ja_1 = zeros(3, 6)
        Ja_1[:,0] = z0

        r0_c2 = k * T2_0 * Matrix([*com2, 1]) # 4x1
        Jl_2 = r0_c2.jacobian(q)

        Ja_2 = zeros(3,6)
        Ja_2[:,0] = z0
        Ja_2[:,1] = z1

        r0_c3 = k * T3_0 * Matrix([*com3 ,1]) # 4x1
        Jl_3 = r0_c3.jacobian(q)
        Ja_3 = zeros(3,6)
        Ja_3[:,0] = z0
        Ja_3[:,1] = z1
        Ja_3[:,2] = z2

        r0_c4 = k * T4_0 * Matrix([*com4 ,1]) # 4x1
        Jl_4 = r0_c4.jacobian(q)
        Ja_4 = zeros(3,6)
        Ja_4[:,0] = z0
        Ja_4[:,1] = z1
        Ja_4[:,2] = z2
        Ja_4[:,3] = z3

        r0_c5 = k * T5_0 * Matrix([*com5 ,1]) # 4x1
        Jl_5 = r0_c5.jacobian(q)
        Ja_5 = zeros(3,6)
        Ja_5[:,0] = z0
        Ja_5[:,1] = z1
        Ja_5[:,2] = z2
        Ja_5[:,3] = z3
        Ja_5[:,4] = z4

        r0_c6 = k * T6_0 * Matrix([*com6 ,1]) # 4x1
        Jl_6 = r0_c6.jacobian(q)
        Ja_6 = zeros(3,6)
        Ja_6[:,0] = z0
        Ja_6[:,1] = z1
        Ja_6[:,2] = z2
        Ja_6[:,3] = z3
        Ja_6[:,4] = z4
        Ja_6[:,5] = z5

        print("Jacob finished")

        # Inertia matrix
        D = sym.simplify(m[1] * Jl_1.T * Jl_1 + Ja_1.T * I1 * Ja_1) + \
            sym.simplify(m[2] * Jl_2.T * Jl_2 + Ja_2.T * I2 * Ja_2) + \
            sym.simplify(m[3] * Jl_3.T * Jl_3 + Ja_3.T * I3 * Ja_3) + \
            sym.simplify(m[4] * Jl_4.T * Jl_4 + Ja_4.T * I4 * Ja_4) + \
            sym.simplify(m[5] * Jl_5.T * Jl_5 + Ja_5.T * I5 * Ja_5) + \
            sym.simplify(m[6] * Jl_6.T * Jl_6 + Ja_6.T * I6 * Ja_6)

        D = D.n(2)
        print("D finished")

        # Compute gravity vector g

        g0 = [0, 0, -9.81] # gravity acceleration vector with respect to frame 0
        Jl = [Jl_1, Jl_2, Jl_3, Jl_4, Jl_5, Jl_6]

        g = zeros(6,1)  # gravity vector
        for i in range(0,6):
            for j in range(i,6):
                g[i] = g[i] + m[j] * g0 * Jl[:,(j-1)*6+i]
       
        g = g.n(2)
        print("g finished")

        # Compute coriolis and centrifugal terms h

        h = zeros(6,1)
        for i in range(0,6):
            for j in range(0,6):
                for k in range(0,6):
                    temp1 = q[k]
                    temp2 = q[i]
                    h[i] = h[i] + (sym.diff(D[i,j],temp1) - 0.5 * sym.diff(D[j,k],temp2))*dq[j]*dq[k]
      
        h = h.n(2)
        print("h finished")

        # Final dynamic equation

        tau = D * ddq + h + g
        tau = tau.n(2)
        print("tau finished")

        # # Generating matlab scripts
        inertia_mat = sym.lambdify(q,D,'numpy')
        gravity_vector = sym.lambdify(q,g,'numpy')
        coriolis_centrifugal_vec = sym.lambdify((q,dq),h,'numpy')

        import inspect
        a = inspect.getsource(inertia_mat)
        b = inspect.getsource(gravity_vector)
        c = inspect.getsource(coriolis_centrifugal_vec)

        with open('i.py','w') as file:
            file.write(a)
        with open('g.py','w') as file:
            file.write(b)
        with open('c.py','w') as file:
            file.write(c)


        print(inertia_mat,gravity_vector,coriolis_centrifugal_vec)

        # matlabFunction(D, 'vars', {q}, 'file', 'inertiaMatrix', 'Optimize', false);
        # matlabFunction(g, 'vars', {q}, 'file', 'gravityVector', 'Optimize', false);
        # matlabFunction(h, 'vars', {q,dq}, 'file', 'cor_centriTerms', 'Optimize', false);

    def equation_of_motion(tau, q, dq):
        # Get joint accelerations dqq by solving forward dynamics equation

        # dqq  = inertiaMatrix(q)\(tau - cor_centriTerms(q,dq));
        pass


if __name__ == "__main__":
    # dynamics = Dynamics()
    # dynamics.init()
    # kom = Kinematics()
    # dynamics.compute_dynamics(kom)
    # print("Finished")

    xml_string = xacro.process_file(r"C:\Chethan\Mechanical\projects\Major_Project\software\MIRS\mirs\src\mirs_description\src\description\robot.urdf").toxml()
    with open('robot.txt','w') as f:
        f.write(xml_string)