import numpy as np

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
    def __init__(self,n=6, theta1=0, theta2=0, theta3=0, theta4=0, theta5=0, theta6=0):
        self.n = n
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.theta5 = theta5
        self.theta6 = theta6
        self.__T = {"0T1":[],
                    "1T2":[],
                    "2T3":[],
                    "3T4":[],
                    "4T5":[],
                    "5T6":[],
                    "0T6":[],
                    }
        self.__R = {
            "0R1":[],
            "1R2":[],
            "2R3":[],
            "3R4":[],
            "4R5":[],
            "5R6":[],
        }
        self.__t = {
            "0t1":[],
            "1t2":[],
            "2t3":[],
            "3t4":[],
            "4t5":[],
            "5t6":[],
        }
    
    def update(self,theta1, theta2, theta3, theta4, theta5, theta6):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.theta5 = theta5
        self.theta6 = theta6
        self.compute()

    def rotation_matrix(self,i):
        return self.__R[f"{i-1}R{i}"]
        
    def displacement_matix(self,i):
        return self.__t[f"{i-1}R{i}"] 
    
    def compute(self):
        # Rotation matrices

        self.__R["0R1"] = np.mat([
            [cos(self.theta1), 0,  sin(self.theta1)],
            [sin(self.theta1), 0, -cos(self.theta1)],
            [0, 1,        0]
        ])

        self.__R["1R2"] = np.mat([
            [cos(self.theta2), -sin(self.theta2),  0],
            [sin(self.theta2),  cos(self.theta2),  0],
            [0,       0,       1]
        ])

        self.__R["2R3"] = np.mat([
            [-sin(self.theta3), 0,  cos(self.theta3)],
            [cos(self.theta3), 0,  sin(self.theta3)],
            [0, 1,        0]
        ])

        self.__R["3R4"] = np.mat([
            [-sin(self.theta4), 0, cos(self.theta4)],
            [cos(self.theta4), 0, sin(self.theta4)],
            [0, 1,        0]
        ])

        self.__R["4R5"] = np.mat([
            [-sin(self.theta5), 0, cos(self.theta5)],
            [cos(self.theta5), 0, sin(self.theta5)],
            [0, 1,       0]
        ])

        self.__R["5R6"] = np.mat([
            [cos(self.theta6), -sin(self.theta6), 0],
            [sin(self.theta6),  cos(self.theta6), 0],
            [0,  0,          1]
        ])

        # Traslation vectors
        self.__t["0t1"] = np.mat([[0],
                            [0],
                            [L1]
                            ])

        self.__t["1t2"] = np.mat([[L2*cos(self.theta2)],
                            [L2*sin(self.theta2)],
                            [0]
                            ])

        self.__t["2t3"] = np.mat([[0],
                            [0],
                            [0]
                            ])

        self.__t["3t4"] = np.mat([[0],
                            [0],
                            [L4]
                            ])

        self.__t["4t5"] = np.mat([[-L5*sin(self.theta5)],
                            [L5*cos(self.theta5)],
                            [0]
                            ])

        self.__t["5t6"] = np.mat([[L6*cos(self.theta6)],
                            [L6*sin(self.theta6)],
                            [0]
                            ])
        
        # Transformation matrices
        self.__T["0T1"] = np.concatenate((self.R0_1, self.d0_1), 1)
        self.__T["1T2"] = np.concatenate((self.R1_2, self.d1_2), 1)
        self.__T["2T3"] = np.concatenate((self.R2_3, self.d2_3), 1)
        self.__T["3T4"] = np.concatenate((self.R3_4, self.d3_4), 1)
        self.__T["4T5"] = np.concatenate((self.R4_5, self.d4_5), 1)
        self.__T["5T6"] = np.concatenate((self.R5_6, self.d5_6), 1)
        self.__T["0T6"] = np.dot(self.__T["0T1"],
                                 self.__T["1T2"],
                                 self.__T["2T3"],
                                 self.__T["3T4"],
                                 self.__T["4T5"],
                                 self.__T["5T6"])
    """ Return k-1_T_k"""
    def T(self,k):
        return self.__T[f"{k}T{k+1}"]

    def homogenous_transformation_matrix(self,i,j):
        Tij = np.eye(4)
        for k in range(i+1,j):
            Tij[:,:] = np.dot(Tij,self.T(k))
        return Tij

    def DH(self,theta,alpha,d,a):
        i_1_T_i = [ [ cos(theta), -sin(theta)*cos(alpha), sin(alpha)*sin(theta), a*cos(theta) ],
                    [ sin(theta),  cos(theta)*cos(alpha),-sin(alpha)*cos(theta), a*sin(theta) ],
                    [    0      ,        sin(alpha)     ,       cos(alpha)     ,       d      ],
                    [    0      ,            0          ,          0           ,       1      ]
                   ]
        

        return i_1_T_i
    
    