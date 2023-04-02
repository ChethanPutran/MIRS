import numpy as np
#import roboticstoolbox as rtb
from  mirs_controller.transformations.matrices import Matrices


class Dynamics:
    def init(self,urdf_file,n_joints=6,gx=0,gy=0,gz=-9.81):
        self.matrices = Matrices()
        self.__I = {
            "I1" : [],
            "I2" : [],
            "I3" : [],
            "I4" : [],
            "I5" : [],
            "I6" : [],
        }
        self.__m = {
            "m1" : 0,
            "m2" : 0,
            "m3" : 0,
            "m4" : 0,
            "m5" : 0,
            "m6" : 0,
        }
        self.__cg = {
            "cg1" : (0,0,0),
            "cg2" : (0,0,0),
            "cg3" : (0,0,0),
            "cg4" : (0,0,0),
            "cg5" : (0,0,0),
            "cg6" : (0,0,0),
        }
        # self.read_inertia_mats(urdf_file)
        self.n_joints = n_joints
        self.__D_i = np.zeros((self.n_joints,1))
        self.__D_ij = np.zeros(self.n_joints)
        self.__D_ijk = np.zeros((self.n_joints,1))
        self.__U_ij= np.zeros((self.n_joints,1))
        self.__U_ijk= np.zeros((self.n_joints,1))
        self.__J_i =  np.zeros((n_joints,4,4))
        self.__g_T = [gx,gy,gz,0]
        self.__T_i = np.zeros((self.n_joints,1))

        # puma = rtb.models.DH.Puma560()
        # tau = puma.rne(puma.qn, np.zeros((6,)), np.zeros((6,)))
        # print(tau)

    def Compute_P_i(self,r_i,i):
        """ 
            -> Returns position of 'i' th link w.r.to base frame\n
            -> r_i - Location of any point on 'i' th link
                w.r.to reference frame in the 'i' th link
        """
        return np.dot(self.matrices.HomogenousTM(0,i),r_i)
    
    def Compute_U_ij(self,i,j):
        """
            Returns del(0_T_i)/del(q_j)
        """
        assert j<=i , "j should be less than or equal to i"

        del_T_by_del_q = np.dot(self.matrices.HomogenousTM(0,j),
                                self.D(),
                                self.matrices.HomogenousTM(j,i))

        return del_T_by_del_q
    
    def Compute_U_ij(self,i,j,k=None):
        """
            Returns del(del(0_T_i)/del(q_j))/del(q_k)
        """
        assert j<=i and  k<=i, "j & k should be less than or equal to i"

        p = j  # min
        q = k  # max

        if(k and j>k):
            # If k is less than l
            p = k
            q = j
        
        del_T_by_del_q = np.dot(self.matrices.HomogenousTM(0,p),
                            self.D(),
                            self.matrices.HomogenousTM(p,q),
                            self.D(),
                            self.matrices.HomogenousTM(q,i))

        return del_T_by_del_q
    
    
    def Compute_J_i(self,i):
        """
            Return inertia tensor of ith link
            integ(r_i*r_i_T*dm_i)
        
        """
        Ixx = self.__I[f"I{i}"][0]
        Iyy = self.__I[f"I{i}"][0]
        Izz = self.__I[f"I{i}"][0]
        Ixy = self.__I[f"I{i}"][0]
        Iyz = self.__I[f"I{i}"][0]
        Ixz = self.__I[f"I{i}"][0]
        m   = self.__m[f"m{i}"]
        x_bar,y_bar,z_bar =self.__cg[f"cg{i}"]

        J_i = [[0.5*(-Ixx+Iyy+Izz),     Ixy ,             Ixz,        m*x_bar],
             [    Ixy,          0.5*(Ixx-Iyy+Izz),      Iyz,        m*y_bar],
             [    Ixz,                Iyz,       0.5*(Ixx+Iyy-Izz), m*z_bar],
             [   m*x_bar,           m*y_bar,          m*z_bar,         1   ]
             ]
        
        return J_i
    
    def Compute_D_i(self,i):
        pass

    def Compute_D_ij(self,i,j):
        """
         It calculates the inertia term

        """
        sum_ = 0

        for p in range(max(i,j),self.n_joints+1):

            # U_pj
            U = self.U_ijk(p,j)

            # Jp
            J = self.J_i(p)

            # U_pi_T
            U_T = self.U_ijk(p,i).T

            sum_ += np.trace(np.dot(U,J,U_T))
        return sum_

    def Compute_D_ijk(self,i,j,k):
        """
         It calculates the centrifugal & coriolis term

        """
        for p in range(max(i,j,k),self.n_joints+1):
            self.__D_ijk[i][p] += 0
        
    def T_i(self,i,q,q_dot,q_dotdot):
        """
            Calculates the 'i' th joint torque required
        
        """

        D_i = []
        D_ij = []
        D_ijk = []
        # Inertia matrix
        M = [self.D_ij(i,j) for j in range(1,self.n_joints+1)]

        A = np.dot(D_ij[i],q_dotdot)
        B = np.dot((np.dot(D_ijk,q_dot).T),q_dot)
        C = D_i[i]
        
        T = A + B + C

    def del_T_by_del_q_i(self,i):
        o_T_i_1= self.matrices.HomogenousTM(0,i-1)
        i_1_T_n = self.matrices.HomogenousTM(i-1,self.n_joints)
        D = self.D(type='revolute')

        del_T_by_del_q = np.dot(o_T_i_1,D,i_1_T_n)

        # or 
        np.dot(self.matrices.HomogenousTM(0,i),self.T_D(self.matrices.HomogenousTM(i-1,i)))

        return del_T_by_del_q

    

    def del_0_T_j_by_del_t(self,j,theta_dot):
        # delT/delq = J
        J = []

        del_T_by_del_t = np.dot(J.T,theta_dot)

        return del_T_by_del_t


    """
    Returns d((i-1)_T_i)/d(q_i) : D

    """
   
    def T_D(self,T,delta):
        """
        Returns the differential transformation of 'i' th frame
        w.r.to 'i-1' th frame due to change 'i' ith joint variable
        
        """

        n = T[:,0]
        o = T[:,0]
        a = T[:,0]
        p = T[:,0]




        T_del_x = np.dot(delta,n)
        T_del_y = np.dot(delta,o)
        T_del_z = np.dot(delta,a)

        #t_d_y = n_x * p_y + n_y * p_x
        T_d_x = -n[0]*p[1] + n[1]*p[0]

        #t_d_y = o_x * p_y + o_y * p_x
        T_d_y = -o[0]*p[1] + o[1]*p[0]

        #t_d_z = a_x * p_y + a_y * p_x
        T_d_z = -a[0]*p[1] + a[1]*p[0]

        t_D = [[     0    , -T_del_z ,  T_del_y , T_d_x],
               [  T_del_z ,     0    , -T_del_x , T_d_y],
               [ -T_del_y ,  T_del_x ,     0    , T_d_z],
               [     0    ,     0    ,     0    ,   0  ],
               ]
        
        # or
        # D_T = np.dot(np.linalg.inv(T),self.D(),T)

        return t_D
        
        
    def D(self,type='revolute'):
        """
        Returns the differential transformation of point in 'i' th frame
        w.r.to 'i' th frame due to change 'i' ith joint variable
        
        """

        if (type == "revolute"):
            # Rotation vector
            K = [0,0,1]
            D = [0,0,0]

        elif (type == "prismatic"):
            # Rotation vector
            K = [0,0,0]
            D = [0,0,1]


        delta_theta = 1
        Kx,Ky,Kz = K
        delta_x,delta_y,delta_z= D

        delta_theta_x = Kx*delta_theta
        delta_theta_y = Ky*delta_theta
        delta_theta_z = Kz*delta_theta

         # For small changes
        D = [
                [       0       , -delta_theta_z ,  delta_theta_y , delta_x],
                [ delta_theta_z ,        0       , -delta_theta_x , delta_y],
                [-delta_theta_y , -delta_theta_x ,       0        , delta_z],
                [       0       ,        0       ,       0        ,    0   ],
            ]
        
        return D
        

    def D_dot(self,theta,theta_dot):
        pass
    def M(self,theta,theta_dotdot):
        sum = 0
        for j in range(1,self.n_joints+1):
            sum+=self.Dij(j)*theta_dotdot[j]

    def C(self,theta,theta_dot):
        pass
    def G(self,theta):
        pass
    def F(self,theta,theta_dot):
        pass
    def compute(self,theta,theta_dot,theta_dotdot):
        # Dynamic equation
        # tau = M*q_dot_dot + C*q_dot + G
        tau = self.M(theta)*theta_dotdot + self.C(theta,theta_dot)*theta_dot + self.G(theta)
        return tau





# Equation  
# syms d1 a1 a2 b1 b2 b4z b5x b5y b5z b6z gs...
#             m1 m2 m3 m4 m5 m6...
#             q1 q2 q3 q4 q5 q6...
#             dq1 dq2 dq3 dq4 dq5 dq6...
#             Ixx1 Ixy1 Ixz1 Iyx1 Iyy1 Iyz1 Izx1 Izy1 Izz1...
#             Ixx2 Ixy2 Ixz2 Iyx2 Iyy2 Iyz2 Izx2 Izy2 Izz2...
#             Ixx3 Ixy3 Ixz3 Iyx3 Iyy3 Iyz3 Izx3 Izy3 Izz3...
#             Ixx4 Ixy4 Ixz4 Iyx4 Iyy4 Iyz4 Izx4 Izy4 Izz4...
#             Ixx5 Ixy5 Ixz5 Iyx5 Iyy5 Iyz5 Izx5 Izy5 Izz5...
#             Ixx6 Ixy6 Ixz6 Iyx6 Iyy6 Iyz6 Izx6 Izy6 Izz6

#         x1 = 0;
#         y1 = 0;
#         z1 = b1;

#         x2 = a1*cos(q1)+b2*cos(q1+q2);
#         y2 = a1*sin(q1)+b2*sin(q1+q2);
#         z2 = d1;

#         x3 = a1*cos(q1)+a2*cos(q1+q2);
#         y3 = a1*sin(q1)+a2*sin(q1+q2);
#         z3 = d1-q3;

#         x4 = x3;
#         y4 = y3;
#         z4 = z3-b4z;

#         x5 = x4+b5x;
#         y5 = y3+b5y;
#         z5 = z4-b5z;

#         x6 = x4;
#         y6 = y4;
#         z6 = z5-b6z;

#         %%%%%%%%%%%%%%%%%%%%%%%% Inertia or Mass Materix %%%%%%%%%%%%%%%%%%%%%%%%%%

#         %Linear Velocity Jacobians COM
#         for i=1:n
#             eval(strcat('J',string(i),'v = sym(zeros(3,n));'))
#             for j=1:n
#             eval(strcat('J',string(i),'v(1,',string(j),') = diff(x',string(i),',q',string(j),');'))%J1v(1,j) = diff(x1,qj)
#             eval(strcat('J',string(i),'v(2,',string(j),') = diff(y',string(i),',q',string(j),');'))%J1v(2,j) = diff(y1,qj)
#             eval(strcat('J',string(i),'v(3,',string(j),') = diff(z',string(i),',q',string(j),');'))%J1v(3,j) = diff(z1,qj)
#             end
#             eval(strcat('J',string(i),'v;'))
#         end

#         %Jacobian of angular velocity of COM
#         Jw = sym(zeros(3,n));
#         for i=1:n
#             if i == 3
#                 Jw((1:3),i)=[0;0;0];%prismatic joint
#             else
#                 Jw((1:3),i)=[0;0;1];%revolute joint
#             end
#             eval(strcat('J',string(i),'w = Jw;'))%Jiw = Jw
            
#         end

#         %Evaluating M matrix
#         M = sym(zeros(n,n));
#         for i=1:n
#             eval(strcat('R',string(i),' = [cos(q',string(i),') -sin(q',string(i),') 0; sin(q',string(i),') cos(q',string(i),') 0; 0 0 1];'))
#             eval(strcat('I',string(i),' = [Ixx',string(i),' Ixy',string(i),' Ixz',string(i),'; Iyx',string(i),' Iyy',string(i),' Iyz',string(i),'; Izx',string(i),' Izy',string(i),' Izz',string(i),'];'))
#             eval(strcat('M',string(i),' = m',string(i),'*transpose(J',string(i),'v)*J',string(i),'v+transpose(J',string(i),'w)*R',string(i),'*I',string(i),'*transpose(R',string(i),')*J',string(i),'w;'))
#             eval(strcat('M = M+M',string(i),';'))
#         end
#         M = simplify(M);
#         fprintf('M matrix symbolic:\n')
#         disp(M)

#         %%%%%%%%%%%%%%%%%%%%%%%%%%% Coriolis forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#         %Evaluating C matrix
#         C = sym(zeros(n,n));
#         for i=1:n
#             for j=1:n
#                 eval(strcat('m',string(i),string(j),' = M(',string(i),',',string(j),');')) %mij = M(i,j)
#                 eval(strcat('c',string(i),string(j),' = sym(0);'))%cij = sym(0)
#                 for k=1:n
#                     eval(strcat('m',string(i),string(k),' = M(',string(i),',',string(k),');'))%mik = M(i,k)
#                     eval(strcat('m',string(j),string(k),' = M(',string(j),',',string(k),');'))%mjk = M(j,k)
                
#                     eval(strcat('c',string(i),string(j),string(k),' = 0.5*(diff(m',string(i),string(j),',q',string(k),')+diff(m',string(i),string(k),',q',string(j),')-diff(m',string(j),string(k),',q',string(i),'));')) 
#                     eval(strcat('c',string(i),string(j),' = c',string(i),string(j),' + c',string(i),string(j),string(k),'*dq',string(k),';'))%cij = cij + cijk*dqk;
#                 end
#                 eval(strcat('C(',string(i),',',string(j),') = c',string(i),string(j),';'))%C(i,j) = cij
#             end
#         end
#         C = simplify(C);
#         fprintf('C matrix symbolic:\n')
#         disp(C)

#         %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gravity forces %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#         %Evaluating g vector
#         g0 = sym([0;-gs;0]);
#         g = sym(zeros(n,1));
#         for i=1:n
#             eval(strcat('g',string(i),' = sym(0);'))%gi = sym(0)
#             for k=1:n
#                 eval(strcat('g',string(i),' = g',string(i),'-transpose(J',string(k),'v((1:3),',string(i),'))*m',string(k),'*g0;'))%gi = gi-transpose(Jkvi((1:3),k))*mk*g0
#             end
#             eval(strcat('g(',string(i),') = g',string(i),';'))%g(i) = gi
#         end
#         g = simplify(g);
#         fprintf('g vector symbolic:\n')
#         disp(g)

#         %%%%%%%%%%%%%%%%%%%%%%%%% Substitution Solution %%%%%%%%%%%%%%%%%%%%%%%%%%%

#         d1 = 0.6;
#         a1 = 0.1;
#         a2 = 0.4;
#         b1 = 0.35;
#         b2 = 0.2;
#         b4z = 0.1;
#         b5x = 0.1;
#         b5y = 0.1;
#         b5z = 0.1;
#         b6z = 0.1;
#         gs = 9.81;
#         m1 = 3; 
#         m2 = 2;
#         m3 = 1.5;
#         m4 = 1;
#         m5 = 0.9;
#         m6 = 0.8;
#         q1 = deg2rad(50);
#         q2 = deg2rad(30);
#         q3 = 0.15;
#         q4 = deg2rad(90);
#         q5 = deg2rad(-30);
#         q6 = deg2rad(10);
#         dq = [0.2; 0.3; 0.15; 0.1; 0.22; 0.18];
#         dq1 = dq(1);
#         dq2 = dq(2);
#         dq3 = dq(3);
#         dq4 = dq(4);
#         dq5 = dq(5);
#         dq6 = dq(6);
#         Izz1 = 0.01;
#         Izz2 = 0.015;
#         Izz3 = 0.001;
#         Izz4 = 0.0015;
#         Izz5 = 0.02;
#         Izz6 = 0.002;

#         M = subs(M);%substitue in symbolic form 
#         C = subs(C);
#         g = subs(g);
#         M = double(M);%convert from sym to double
#         C= double(C);
#         g = double(g);

#         fprintf('M matrix:\n')
#         disp(M)
#         fprintf('C matrix:\n')
#         disp(C)
#         fprintf('g matrix:\n')
#         disp(g)

#         %%%%%%%%%%%%%%%%%%%%%%%%%%% Torque Calculation %%%%%%%%%%%%%%%%%%%%%%%%%%%%

#         fprintf('Dynamics Equation: M(q)ddq + C(q,dq)dq + g(q) = T\n')
#         ddq = [0.1; 0.15; 0.1; 0.06; 0.07; 0.12];
#         T = M*ddq + C*dq + g; 
#         fprintf('Desired Torques:\n')
#         disp(T)
