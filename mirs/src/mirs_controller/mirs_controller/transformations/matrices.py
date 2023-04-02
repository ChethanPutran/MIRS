import numpy as np

pi = np.pi
cos = np.cos
sin = np.sin

from mirs_controller.common.config import LINK_1_LENGTH,LINK_2_LENGTH,LINK_3_LENGTH,LINK_4_LENGTH,LINK_5_LENGTH,LINK_6_LENGTH

class Matrices:
    n = 6
    theta1 = 0.0
    theta2 = 0.0
    theta3 = 0.0
    theta4 = 0.0
    theta5 = 0.0
    theta6 = 0.0
    __T = {}
    __R = {}
    __t = {}

    @staticmethod
    def init(n=6, theta1=0, theta2=0, theta3=0, theta4=0, theta5=0, theta6=0):
        Matrices.n = n
        Matrices.theta1 = theta1
        Matrices.theta2 = theta2
        Matrices.theta3 = theta3
        Matrices.theta4 = theta4
        Matrices.theta5 = theta5
        Matrices.theta6 = theta6
        Matrices.__T = {"0T1":[],
                    "1T2":[],
                    "2T3":[],
                    "3T4":[],
                    "4T5":[],
                    "5T6":[],
                    "0T6":[],
                    }
        Matrices.__R = {
            "0R1":[],
            "1R2":[],
            "2R3":[],
            "3R4":[],
            "4R5":[],
            "5R6":[],
        }
        Matrices.__t = {
            "0t1":[],
            "1t2":[],
            "2t3":[],
            "3t4":[],
            "4t5":[],
            "5t6":[],
        }
    
    @staticmethod
    def get_ee_pose():
        return Matrices.__T['0T6']
    
    @staticmethod
    def update(theta1, theta2, theta3, theta4, theta5, theta6):
        Matrices.theta1 = theta1
        Matrices.theta2 = theta2
        Matrices.theta3 = theta3
        Matrices.theta4 = theta4
        Matrices.theta5 = theta5
        Matrices.theta6 = theta6
        Matrices.compute()

    @staticmethod
    def rotation_matrix(i):
        return Matrices.__R[f"{i-1}R{i}"]
    
    @staticmethod
    def displacement_matix(i):
        return Matrices.__t[f"{i-1}R{i}"] 
    
    @staticmethod
    def compute():
        # Rotation matrices

        Matrices.__R["0R1"] = np.mat([
            [cos(Matrices.theta1), 0,  sin(Matrices.theta1)],
            [sin(Matrices.theta1), 0, -cos(Matrices.theta1)],
            [0, 1,        0]
        ])

        Matrices.__R["1R2"] = np.mat([
            [cos(Matrices.theta2), -sin(Matrices.theta2),  0],
            [sin(Matrices.theta2),  cos(Matrices.theta2),  0],
            [0,       0,       1]
        ])

        Matrices.__R["2R3"] = np.mat([
            [-sin(Matrices.theta3), 0,  cos(Matrices.theta3)],
            [cos(Matrices.theta3), 0,  sin(Matrices.theta3)],
            [0, 1,        0]
        ])

        Matrices.__R["3R4"] = np.mat([
            [-sin(Matrices.theta4), 0, cos(Matrices.theta4)],
            [cos(Matrices.theta4), 0, sin(Matrices.theta4)],
            [0, 1,        0]
        ])

        Matrices.__R["4R5"] = np.mat([
            [-sin(Matrices.theta5), 0, cos(Matrices.theta5)],
            [cos(Matrices.theta5), 0, sin(Matrices.theta5)],
            [0, 1,       0]
        ])

        Matrices.__R["5R6"] = np.mat([
            [cos(Matrices.theta6), -sin(Matrices.theta6), 0],
            [sin(Matrices.theta6),  cos(Matrices.theta6), 0],
            [0,  0,          1]
        ])

        # Traslation vectors
        Matrices.__t["0t1"] = np.mat([[0],
                            [0],
                            [LINK_1_LENGTH]
                            ])

        Matrices.__t["1t2"] = np.mat([[LINK_2_LENGTH*cos(Matrices.theta2)],
                            [LINK_2_LENGTH*sin(Matrices.theta2)],
                            [0]
                            ])

        Matrices.__t["2t3"] = np.mat([[0],
                            [0],
                            [0]
                            ])

        Matrices.__t["3t4"] = np.mat([[0],
                            [0],
                            [LINK_4_LENGTH]
                            ])

        Matrices.__t["4t5"] = np.mat([[-LINK_4_LENGTH*sin(Matrices.theta5)],
                            [LINK_5_LENGTH*cos(Matrices.theta5)],
                            [0]
                            ])

        Matrices.__t["5t6"] = np.mat([[LINK_6_LENGTH*cos(Matrices.theta6)],
                            [LINK_6_LENGTH*sin(Matrices.theta6)],
                            [0]
                            ])
        
        # Transformation matrices
        Matrices.__T["0T1"] = np.concatenate((Matrices.R0_1, Matrices.d0_1), 1)
        Matrices.__T["1T2"] = np.concatenate((Matrices.R1_2, Matrices.d1_2), 1)
        Matrices.__T["2T3"] = np.concatenate((Matrices.R2_3, Matrices.d2_3), 1)
        Matrices.__T["3T4"] = np.concatenate((Matrices.R3_4, Matrices.d3_4), 1)
        Matrices.__T["4T5"] = np.concatenate((Matrices.R4_5, Matrices.d4_5), 1)
        Matrices.__T["5T6"] = np.concatenate((Matrices.R5_6, Matrices.d5_6), 1)
        Matrices.__T["0T6"] = np.dot(Matrices.__T["0T1"],
                                 Matrices.__T["1T2"],
                                 Matrices.__T["2T3"],
                                 Matrices.__T["3T4"],
                                 Matrices.__T["4T5"],
                                 Matrices.__T["5T6"])
   
   
    """ Return k-1_T_k"""
    @staticmethod
    def T(k):
        return Matrices.__T[f"{k}T{k+1}"]

    @staticmethod
    def homogenous_transformation_matrix(i,j):
        Tij = np.eye(4)
        for k in range(i+1,j):
            Tij[:,:] = np.dot(Tij,Matrices.T(k))
        return Tij

    @staticmethod
    def DH(theta,alpha,d,a):
        i_1_T_i = [ [ cos(theta), -sin(theta)*cos(alpha), sin(alpha)*sin(theta), a*cos(theta) ],
                    [ sin(theta),  cos(theta)*cos(alpha),-sin(alpha)*cos(theta), a*sin(theta) ],
                    [    0      ,        sin(alpha)     ,       cos(alpha)     ,       d      ],
                    [    0      ,            0          ,          0           ,       1      ]
                   ]
        

        return i_1_T_i
    
    