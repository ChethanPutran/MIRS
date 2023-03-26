import numpy as np
from ikpy.chain import Chain

pi = np.pi
cos = np.cos
sin = np.sin

L1 = 150
L2 = 200
L3 = 200
L4 = 45
L5 = 35
L6 = 25


class Matrices:
    def __init__(self, theta1=0, theta2=0, theta3=0, theta4=0, theta5=0, theta6=0):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.theta5 = theta5
        self.theta6 = theta6

    def RotationMatrices(self):
        self.R0_1 = np.mat([
            [cos(self.theta1), 0,  sin(self.theta1)],
            [sin(self.theta1), 0, -cos(self.theta1)],
            [0, 1,        0]
        ])

        self.R1_2 = np.mat([
            [cos(self.theta2), -sin(self.theta2),  0],
            [sin(self.theta2),  cos(self.theta2),  0],
            [0,       0,       1]
        ])

        self.R2_3 = np.mat([
            [-sin(self.theta3), 0,  cos(self.theta3)],
            [cos(self.theta3), 0,  sin(self.theta3)],
            [0, 1,        0]
        ])

        self.R3_4 = np.mat([
            [-sin(self.theta4), 0, cos(self.theta4)],
            [cos(self.theta4), 0, sin(self.theta4)],
            [0, 1,        0]
        ])

        self.R4_5 = np.mat([
            [-sin(self.theta5), 0, cos(self.theta5)],
            [cos(self.theta5), 0, sin(self.theta5)],
            [0, 1,       0]
        ])

        self.R5_6 = np.mat([
            [cos(self.theta6), -sin(self.theta6), 0],
            [sin(self.theta6),  cos(self.theta6), 0],
            [0,  0,          1]
        ])

    def DisplacementMatices(self):
        self.d0_1 = np.mat([[0],
                            [0],
                            [L1]
                            ])

        self.d1_2 = np.mat([[L2*cos(self.theta2)],
                            [L2*sin(self.theta2)],
                            [0]
                            ])

        self.d2_3 = np.mat([[0],
                            [0],
                            [0]
                            ])

        self.d3_4 = np.mat([[0],
                            [0],
                            [L4]
                            ])

        self.d4_5 = np.mat([[-L5*sin(self.theta5)],
                            [L5*cos(self.theta5)],
                            [0]
                            ])

        self.d5_6 = np.mat([[L6*cos(self.theta6)],
                            [L6*sin(self.theta6)],
                            [0]
                            ])

    def HomogenousTM(self):
        self.H0_1 = np.concatenate((self.R0_1, self.d0_1), 1)
        self.H1_2 = np.concatenate((self.R1_2, self.d1_2), 1)
        self.H2_3 = np.concatenate((self.R2_3, self.d2_3), 1)
        self.H3_4 = np.concatenate((self.R3_4, self.d3_4), 1)
        self.H4_5 = np.concatenate((self.R4_5, self.d4_5), 1)
        self.H5_6 = np.concatenate((self.R5_6, self.d5_6), 1)

        self.H0_6 = np.dot(self.H0_1, self.H1_2, self.H2_3,
                           self.H3_4, self.H4_5, self.H5_6)

        x, y, z, _ = self.H0_6[:, -1]


class Kinematics:
    def __init__(self):
        pass

    def forward(self):
        pass

    def inverse(self,x,y,z,theta,beta,gamma):
        
        #Using numerical computation
        IKPY_MAX_ITERATIONS = 4
        filename = None
        armChain = Chain.from_urdf_file(filename)

        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0,0,0,0,0]
        ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

        return ikResults
