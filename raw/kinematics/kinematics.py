import numpy as np
import sympy as sym
from sympy import Symbol,Matrix,zeros,eye,cos,sin,pi
#from ikpy.chain import Chain
from mirs_controller.transformations.matrices import Matrices
# pi = np.pi
# cos = np.cos
# sin = np.sin

L1 = 150
L2 = 200
L3 = 200
L4 = 45
L5 = 35
L6 = 25


class Kinematics:
    def __init__(self):
        self.matrices = Matrices
        self.computed = False

    def forward(self,theta):
        return np.dot(self.matrices.get_ee_pose(),theta)
        
    def inverse(self,x,y,z,theta,beta,gamma):
        
        # Define the desired position of the end effector
        # This is the target (goal) location.
        x = 4.0
        y = 2.0
        
        # Calculate the angle of the second joint
        theta_2 = np.arctan2(y,x)
        print(f'Theta 2 = {theta_2} radians\n')
        
        # Define the desired orientation of the end effector relative to the base frame 
        # (i.e. global frame)
        # This is the target orientation.
        # The 3x3 rotation matrix of frame 6 relative to frame 0
        rot_mat_0_6 = np.array([[-1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0],
                                [0.0, 0.0, 1.0]])
        
        # The 3x3 rotation matrix of frame 3 relative to frame 0
        rot_mat_0_3 = np.array([[-np.sin(theta_2), 0.0, np.cos(theta_2)],
                                [np.cos(theta_2), 0.0, np.sin(theta_2)],
                                [0.0, 1.0, 0.0]])
        
        # Calculate the inverse rotation matrix
        inv_rot_mat_0_3 = np.linalg.inv(rot_mat_0_3)
        
        # Calculate the 3x3 rotation matrix of frame 6 relative to frame 3
        rot_mat_3_6 = inv_rot_mat_0_3 @ rot_mat_0_6
        print(f'rot_mat_3_6 = {rot_mat_3_6}')
        
        # We know the equation for rot_mat_3_6 from our pencil and paper
        # analysis. The simplest term in that matrix is in the third column,
        # third row. The value there in variable terms is cos(theta_5).
        # From the printing above, we know the value there. Therefore, 
        # cos(theta_5) = value in the third row, third column of rot_mat_3_6, which means...
        # theta_5 = arccosine(value in the third row, third column of rot_mat_3_6)
        theta_5 = np.arccos(rot_mat_3_6[2, 2])
        print(f'\nTheta 5 = {theta_5} radians')
        
        # Calculate the value for theta_6
        # We'll use the expression in the third row, first column of rot_mat_3_6.
        # -sin(theta_5)cos(theta_6) = rot_mat_3_6[2,0]
        # Solving for theta_6...
        # rot_mat_3_6[2,0]/-sin(theta_5) = cos(theta_6)
        # arccosine(rot_mat_3_6[2,0]/-sin(theta_5)) = theta_6
        theta_6 = np.arccos(rot_mat_3_6[2, 0] / -np.sin(theta_5))
        print(f'\nTheta 6 = {theta_6} radians')
        
        # Calculate the value for theta_4 using one of the other
        # cells in rot_mat_3_6. We'll use the second row, third column.
        # cos(theta_4)sin(theta_5) = rot_mat_3_6[1,2]
        # cos(theta_4) = rot_mat_3_6[1,2] / sin(theta_5)
        # theta_4 = arccosine(rot_mat_3_6[1,2] / sin(theta_5))
        theta_4 = np.arccos(rot_mat_3_6[1,2] / np.sin(theta_5))
        print(f'\nTheta 4 = {theta_4} radians')
        
        # Check that the angles we calculated result in a valid rotation matrix
        r11 = -np.sin(theta_4) * np.cos(theta_5) * np.cos(theta_6) - np.cos(theta_4) * np.sin(theta_6)
        r12 = np.sin(theta_4) * np.cos(theta_5) * np.sin(theta_6) - np.cos(theta_4) * np.cos(theta_6)
        r13 = -np.sin(theta_4) * np.sin(theta_5)
        r21 = np.cos(theta_4) * np.cos(theta_5) * np.cos(theta_6) - np.sin(theta_4) * np.sin(theta_6)
        r22 = -np.cos(theta_4) * np.cos(theta_5) * np.sin(theta_6) - np.sin(theta_4) * np.cos(theta_6)
        r23 = np.cos(theta_4) * np.sin(theta_5)
        r31 = -np.sin(theta_5) * np.cos(theta_6)
        r32 = np.sin(theta_5) * np.sin(theta_6)
        r33 = np.cos(theta_5)
        
        check_rot_mat_3_6 = np.array([[r11, r12, r13],
                                    [r21, r22, r23],
                                    [r31, r32, r33]])
        
        # Original matrix
        print(f'\nrot_mat_3_6 = {rot_mat_3_6}')
        
        # Check Matrix
        print(f'\ncheck_rot_mat_3_6 = {check_rot_mat_3_6}\n')
        
        # Return if Original Matrix == Check Matrix
        rot_minus_check_3_6 = rot_mat_3_6.round() - check_rot_mat_3_6.round()
        zero_matrix = np.array([[0, 0, 0],
                                [0, 0, 0],
                                [0, 0, 0]])
        matrices_are_equal = np.array_equal(rot_minus_check_3_6, zero_matrix)
        
        # Determine if the solution is valid or not
        # If the solution is valid, that means the end effector of the robotic 
        # arm can reach that target location.
        if (matrices_are_equal):
            valid_matrix = "Yes"
        else:
            valid_matrix = "No" 
        print(f'Is this solution valid?\n{valid_matrix}\n')
        
    def numerical_inverse(self):
        #Using numerical computation
        IKPY_MAX_ITERATIONS = 4
        filename = None
        armChain = Chain.from_urdf_file(filename)

        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0,0,0,0,0]
        ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
 
    def jacobian(self):
        J = np.zeros((6,self.n))


        for i in range(0,self.n):
            T = self.matrices.homogenous_transformation_matrix(i-1,self.n)

            n = T[:,0]
            o = T[:,1]
            a = T[:,2]
            p = T[:,3]

            #t_d_y = n_x * p_y + n_y * p_x
            i_d_x = -n[0]*p[1] + n[1]*p[0]

            #t_d_y = o_x * p_y + o_y * p_x
            i_d_y = -o[0]*p[1] + o[1]*p[0]

            #t_d_z = a_x * p_y + a_y * p_x
            i_d_z = -a[0]*p[1] + a[1]*p[0]
            
            # i_del_x = n_z
            i_del_x = n[2]

            # i_del_y =  o_z
            i_del_y = o[2]

            # i_del_z = a_z
            i_del_z = a[2]

            J[:,i] = np.array([i_d_x,i_d_y,i_d_z,i_del_x,i_del_y,i_del_z]).reshape(1,-1)

        return J

    def compute_ee_velocity(self,theta_dot):
        return np.dot(self.matrices.Jacobian(),theta_dot)
    
    def compute_joint_velocity(self,ee_velocity):
        J = self.matrices.Jacobian()

        if self.n_joints < 6:
            # Compute pseudo inverse
            J_p_inv = np.dot(np.linalg.inv(J.T,J),J.T)

            return np.dot(J_p_inv,ee_velocity)

        # Actual inverse (6x6)
        J_inv = np.linalg.inv(J)
        return np.dot(J_inv,ee_velocity)

    def DH(self,alpha, a, d, theta):
            
        a11 = cos(theta)
        a12 = -sin(theta)*round(cos(alpha),2)
        a13 = sin(theta)*round(sin(alpha),2)
        a14 = a*cos(theta)

        a21 = sin(theta)
        a22 = cos(theta)*round(cos(alpha),2)
        a23 = -cos(theta)*round(sin(alpha),2)
        a24 = a*sin(theta)

        a31 = 0
        a32 = round(sin(alpha),2)
        a33 = round(cos(alpha),2)
        a34 =d

        a41 = 0
        a42 = 0
        a43 = 0
        a44 = 1


        trans_mat = Matrix([[a11, a12, a13, a14],
            [a21, a22, a23, a24],
            [a31, a32, a33, a34],
            [a41, a42, a43, a44]])
      
        return trans_mat

    def compute_kinematics(self):
        self.computed = True

        #Forward kinematics and forward differential kinematics analysis of 6 DOF MIR

        n = 6

        q1 = Symbol('q1')
        q2 = Symbol('q2')
        q3 = Symbol('q3')
        q4 = Symbol('q4')
        q5 = Symbol('q5')
        q6 = Symbol('q6')
        D1 = Symbol('D1')
        D2 = Symbol('D2')
        D3 = Symbol('D3')
        D4 = Symbol('D4')
        D5 = Symbol('D5')
        D6 = Symbol('D6')
        e2 = Symbol('e2')
        aa = Symbol('aa')
        d4b = Symbol('d4b')
        d5b = Symbol('d5b')
        d6b = Symbol('d6b')
       
        D1 = 0.2755
        D2 = 0.41
        D3 = 0.2073
        D4 = 0.0741
        D5 = 0.0741
        D6 = 0.16
        e2 = 0.0098
        aa = pi/6
        d4b = D3 + sin(aa)/sin(2*aa)*D4
        d5b = sin(aa)/sin(2*aa)*(D4+D5)
        d6b = sin(aa)/sin(2*aa)*D5 + D6
       

        #Forward kinematics
        #Classic Denavit Hartenberg parameters
        alpha = [pi/2, pi, pi/2, pi/3, pi/3, pi]
        a = [0, D2, 0, 0, 0, 0]
        d = [D1, 0, -e2, -d4b, -d5b, -d6b]
        theta = [-q1, q2+pi/2, q3-pi/2, q4, q5+pi, q6-pi/2]

        #Compute DH matrices
        A = zeros(4,n*4)
        
        A[:,0:4] = sym.simplify(self.DH(alpha[0], a[0], d[0], theta[0]))

        for i in range(1,6):
            A[:,4*i:4*(i+1)] = sym.simplify(self.DH(alpha[i], a[i], d[i], theta[i]))
        
        print('DH completed')
        
        #Compute relative homogeneous transformation matrices
        self.T1_0 = A[:,0:4]
        self.T2_1 = A[:,4:8]
        self.T3_2 = A[:,8:12]
        self.T4_3 = A[:,12:16]
        self.T5_4 = A[:,16:20]
        self.T6_5 = A[:,20:24]

        #Compute kinematic equation
        self.T6_0 = self.T1_0 * self.T2_1 * self.T3_2 * self.T4_3 * self.T5_4 * self.T6_5 #homogeneous transformation matrix from frame 6 to frame 0
        self.T6_0 = sym.simplify(self.T6_0)


        #End effector's rotation matrix
        r11 = self.T6_0[0,0]
        r21 = self.T6_0[1,0]
        r31 = self.T6_0[2,0]

        r12 = self.T6_0[0,1]
        r22 = self.T6_0[1,1]
        r32 = self.T6_0[2,1]

        r13 = self.T6_0[0,2]
        r23 = self.T6_0[1,2]
        r33 = self.T6_0[2,2]

        #End effector's position vector
        px = self.T6_0[0,3]
        py = self.T6_0[1,3]
        pz = self.T6_0[2,3]

        print('Compute kinematics completed.')


    def forward_differential_kinematics(self):
        if not self.computed:
            self.compute_kinematics()


        #Forward differential kinematics
        #Compute the homogeneous transformation matrices from frame i to inertial frame 0
        self.T1_0 = self.T1_0
        self.T2_0 = sym.simplify(self.T1_0*self.T2_1)
        self.T3_0 = sym.simplify(self.T2_0*self.T3_2)
        self.T4_0 = sym.simplify(self.T3_0*self.T4_3)
        self.T5_0 = sym.simplify(self.T4_0*self.T5_4)
        self.T6_0 = sym.simplify(self.T5_0*self.T6_5)

        #Extract position vectors from each homogeneous transformation matrix
        # p0 = zeros(3,1) 
        # p1 = self.T1_0[0:3,3]
        # p2 = self.T2_0[0:3,3]
        # p3 = self.T3_0[0:3,3]
        # p4 = self.T4_0[0:3,3]
        # p5 = self.T5_0[0:3,3]
        # p6 = self.T6_0[0:3,3]

        #Define vectors around which each link rotates in the precedent coordinate frame
        z0 = Matrix([0,0,1])  #3x1

        z1 = self.T1_0[0:3,2]
        z2 = self.T2_0[0:3,2]
        z3 = self.T3_0[0:3,2]
        z4 = self.T4_0[0:3,2]
        z5 = self.T5_0[0:3,2]
        # z6 = self.T6_0[0:3,2]

        # print("Jacob started!")
        # #Jacobian matrix
        # j11 = z0.cross(p6-p0)
        # j12 = z1.cross(p6-p1)
        # j13 = z2.cross(p6-p2)
        # j14 = z3.cross(p6-p3)
        # j15 = z4.cross(p6-p4)
        # j16 = z5.cross(p6-p5)

        # j21 = z0
        # j22 = z1
        # j23 = z2
        # j24 = z3
        # j25 = z4
        # j26 = z5

        # J = sym.simplify(Matrix([[j11, j12, j13, j14, j15, j16],
        #                 [j21, j22, j23, j24, j25, j26]]))  
        
        # # Rounding
        # J = J.n(2)
                
        # #Extract linear jacobian
        # Jl = J[0:3,:]

        # #Extract angular jacobian
        # Ja = J[3:,:]
        
        # print("Jascob finished")

        return z0,z1,z2,z3,z4,z5

    def get_diff_transforms(self):
        return self.T1_0 ,self.T2_0 ,self.T3_0 ,self.T4_0,self.T5_0 ,self.T6_0 
