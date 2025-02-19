
import numpy as np
import sympy as sp
from sympy import symbols, Matrix, simplify, diff
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame


### Using Eularian approach
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


### Using Eularian approach
def simple_dynamic_equation(transforms, inertias, masses, lengths, joint_angles, joint_velocities, joint_accelerations):
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


### Using Lagrangian approach
# 1.


def convert_to_homegenous(mat):
    a_0,a_1,a_2 = mat
    return [[a_0],[a_1],[a_2],[1]]

def calculate_dynamic_equation(theta, ):
    # theta: joint angles (symbolic)
    # d: link offsets
    # a: link lengths
    # alpha: link twists
    # m: link masses
    # r: link center of masses
    # I: link inertias

    d = dynamicsymbols('d1:7')
    a = dynamicsymbols('a1:7')
    alpha = dynamicsymbols('alpha1:7')
    m = dynamicsymbols('m1:7')
    r = [] 
    I = []  # W.r.to body attached frame

    num_joints = 6

    # Initialize symbolic variables
    q1, q2, q3, q4, q5, q6 = dynamicsymbols('q1:7')
    theta = [q1, q2, q3, q4, q5, q6]
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


    J = []

    # Joint positions and velocities w.r.to base frame
    p = []
    v = []
    for i in range(num_joints):
        T_i = sp.eye(4)
        for j in range(i+1):
            T_i = T_i * T[j]

        p_i = T_i[:3, 3]
        v_i = sp.diff(p_i, q[0])*q_dot[0]   #dp/dt = dp/dq * dq/dt = dp/dq * dq_dot
        for j in range(1, i+1):
            v_i += sp.diff(p_i, q[j])*q_dot[j]

        p.append(p_i)
        v.append(v_i)

    # Or
    J_v,J_w = J
    v = J_v*q_dot
    w = J_w*q_dot

    # Kinetic energy
    K = 0
    for i in range(num_joints):
        T_0_i = sp.eye(4)
        r_i = r[i]

        for j in range(i+1):
            T_0_i = T_0_i * T[j]

        # p_i = p[i]
        # r_i - Position vector of center of mass of ith link w.r.to body attached frame
        # p_i - Position vector of center of mass of ith link w.r.to base reference frame
        # T_0_i - Transformation matrix of ith frame w.r.to base reference frame

        p_i = T_0_i * r_i  

        #v_i = d(p_i)/dt = sum j=1 to i ((del(T_0_i)/del(q_j)) * del(q_j)/dt )*r_i
        # del(T_0_i)/del(q_j) = u_ij
        for j in range(0,i):
            v_i += sp.diff(T_0_i,q[j])*q_dot[j]
        v_i = v_i*r_i

        R_i = T[i][:3, :3]
        v_i = v[i]
        I_i = sp.Matrix(I[i])

        # # Calculate the angular velocity
        # omega_i = np.dot(np.linalg.inv(R_i), q_dot[i] * np.array([0, 0, link_twists[i]]))

        # # Calculate the linear velocity
        # v_i = np.cross(omega_i, p_i)
        # sp.dot(v_i, v_i) same as v.T *v

        #kinetic_energy = kinetic_energy_trans + kinetic_energy_rot
        # omega_I = 

        K += 0.5 * m[i] * sp.dot(v_i, v_i) + 0.5 * sp.dot(sp.dot(v_i, R_i), I_i * sp.transpose(sp.dot(R_i, v_i)))


    # Potential energy
    P = 0
    g_x = g_y = 0
    g_z = -9.8
    g = [g_x,g_y,g_z,0]
    for i in range(num_joints):
        T_0_i = sp.eye(4)
        r_i = r[i] # 3x1
        r_i_bar = convert_to_homegenous(r_i) # 4x1

        for j in range(i+1):
            T_0_i = T_0_i * T[j]

        # p_i = p[i]
        # r_i - Position vector of center of mass of ith link w.r.to body attached frame
        # p_i - Position vector of center of mass of ith link w.r.to base reference frame
        # T_0_i - Transformation matrix of ith frame w.r.to base reference frame

        p_i = T_0_i * r_i_bar  # 4x1

        # Calculate the potential energy of the link
        # p_ci = p_i * (R_i*p_i_ci) 

        # potential_energy -= m[i] * g * p_i[2]  
        # potential_energy -= m[i] * g * p_i[2]  
        #P -= m[i] * np.dot(g, p_i) # m_i * g.T *p_0_ci 
        P -= m[i] * np.dot(g, p_i) # Same as m_i * g.T *p_0_ci   # 1x4 * 4x4 * 4x1 - 1x1


    # Lagrangian
    L = K - P   # K(q,q_dot)  , P(q)

    # Lagrange's equations
    lagrange_equations = []

    # Calculate generalized forces
    tau = Matrix([dynamicsymbols('tau%i' % (i + 1)) for i in range(len(q))])

    for i in range(num_joints):
        dL_dq_dot = sp.diff(L, q_dot[i])
        ddL_dq_dot_dt = sp.diff(dL_dq_dot, q[i]) * q_dot[i] + dL_dq_dot.diff(q_dot[i])
        #ddL_dq_dot_dt = diff(dL_dq_dot, symbols('t'))
        dL_dq= sp.diff(L, q[i])

        lagrange_eqs = Matrix([simplify(ddL_dq_dot_dt) - dL_dq + tau[i] for i in range(len(q))])
        lagrange_equations.append(lagrange_eqs)

        # Simplified expression
        # Since K(q,q_dot) & P(q) eq. can be simplified
        # del_K_del_q_dot = sp.diff(K,q_dot)
        # d_del_K_del_q_dot_dt = sp.diff(del_K_del_q_dot,symbols('t'))
        # del_K_del_q = sp.diff(K,q)
        # del_P_del_q = sp.diff(P,q)

        # tau = d_del_K_del_q_dot_dt - del_K_del_q + del_P_del_q

    return lagrange_equations

# 2.
def calculate_dynamic_equation(transform_matrices, inertial_matrices, joint_angles, joint_velocities, joint_accelerations, gravity):
    num_joints = len(joint_angles)
    M = np.zeros((num_joints, num_joints))
    C = np.zeros((num_joints, num_joints))
    G = np.zeros(num_joints)
    tau = np.zeros(num_joints)

    for i in range(num_joints):
        # Calculate the rotational transformation matrix from frame i to the base frame
        R_i = transform_matrices[i][:3, :3]

        # Calculate the linear transformation matrix from frame i to the base frame
        p_i = transform_matrices[i][:3, 3]

        # Calculate the angular velocity vector
        omega_i = R_i.dot(joint_velocities[i])

        # Calculate the linear velocity vector
        v_i = np.cross(omega_i, p_i)

        # Calculate the kinetic energy
        kinetic_energy = 0.5 * np.transpose(joint_velocities[i]).dot(inertial_matrices[i]).dot(joint_velocities[i])

        # Calculate the potential energy
        potential_energy = -np.transpose(joint_angles[i]).dot(gravity)

        # Calculate the Lagrangian
        lagrangian = kinetic_energy - potential_energy

        # Calculate the partial derivatives of the Lagrangian
        dL_dq = np.zeros(num_joints)
        dL_dq_dot = np.zeros(num_joints)

        dL_dq[i] = np.diff(lagrangian, joint_angles[i])
        dL_dq_dot[i] = np.diff(lagrangian, joint_velocities[i])

        # Calculate the inertia matrix
        M[i, i] = np.diff(dL_dq_dot[i], joint_velocities[i])

        # Calculate the Coriolis matrix
        for j in range(num_joints):
            C[i, j] = np.diff(dL_dq_dot[i], joint_velocities[j])

        # Calculate the gravity vector
        G[i] = np.diff(lagrangian, joint_angles[i])

    # Calculate the joint torques
    tau = M.dot(joint_accelerations) + C.dot(joint_velocities) + G

    return tau

# Example usage
transform_matrices = [...] # Transformation matrices from base to each joint
inertial_matrices = [...] # Inertial matrices of each joint
joint_angles = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) # Joint angles
joint_velocities = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) # Joint velocities
joint_accelerations = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) # Joint accelerations
gravity = np.array([0, 0, -9.81]) # Gravity vector


# Calculate joint torques
torques = calculate_dynamic_equation(transform_matrices, inertial_matrices, joint_angles, joint_velocities, joint_accelerations, gravity)
print("Joint torques:", torques)
