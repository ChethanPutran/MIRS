import numpy as np
import math


class Kinematics:
    def __init__(self, transform, precision=4):
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

    def get_ee_velocity(self, theta_d):
        return self.jacobian() @ theta_d

    def get_joint_velocity(self, v):
        J = self.jacobian()

        # Calculate J Psuedo Jacobian for DOF < 6
        if J.shape[1] < 6:
            J_psuedo = np.linalg.inv(J.T @ J) @ J.T
            return J_psuedo @ v

        return J @ v

    def get_current_joint_state(self):
        return self.transform.get_current_joint_state()

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
                delT_delqi[:3, :3] @ self.transform._0T(self.transform.n).T[:3, :3])

        return self.J

    def forward(self, theta):
        T = self.transform.compute(update=False, theta=theta)
        return T[:3, 3]

    def get_transform(self):
        return self.transform

    def geometric_inverse(self, pt, deg=False, get_best_orientation=True):
        theta = np.zeros((self.transform.n, 2))
        x, y, z = pt
        l0, l1, l2 = self.transform.robot_model.link_lengths
        theta1 = math.atan2(y, x)
        # print("Theta1 :", theta1)
        th3_val = np.round(
            (x**2 + y**2 + (z-l0)**2 - l1**2 - l2**2)/(2*l1*l2), 4)
        # print("th3_val :", th3_val)

        if (th3_val > 1):
            print("Goal is out of reach of robot")
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

    def inverse(self, pt, deg=False, get_best_orientation=True):

        return True, None

    def get_closest_orientation(self):
        pass

    def get_collision_free_orientation(self):
        # NN
        # input current orientation
        # Goal point
        # Objects (image)
        """if there is a choice, our algorithm can choose the solution closest in joint-space.
        However, the notion of "close" might be defined in several ways. For example,
        typical robots could have three large links followed by three smaller, orienting
        linksnear the end-effector. In this case, weights might be applied in the calculation of
        which solution is "closer" so that the selection favors moving smaller joints rather
        than moving the large joints, when a choice exists. The presence of obstacles might
        force a "farther" solution to be chosen in cases where the "closer" solution wouldcause 
        a collision—in general, then, we need to be able to calculate all the possiblesolutions

        the maximum number of solutions is related to how many of the link length parameters
        (the a_i) are zero. The more that are nonzero, the bigger is the maximum number of solutions.

        For a completely general rotary-jointed manipulator with six degrees of freedom,
        there are up to sixteen solutions possible 

        A manipulator wifi be considered solvable if the joint variables can be
        determined by an algorithm that allows one to determine all the sets of joint
        variables associated with a given position and orientation [
        """



# Manipulator solution strategies 
# 1. Closed-form solutions
# 2. Numerical solution

# Closed-form solutions
#  Two methods of obtaining the solution: 
#  1. Algebraic method
#  2. Geometric method