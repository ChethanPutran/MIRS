import numpy as np
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from enum import Enum
import time

GRAVITY = 9.81


class Point:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def to_numpy(self):
        return np.array([
            [self.x],
            [self.y],
            [self.z]]
        )
    def __repr__(self):
        return f"Point(%.2f,%.2f,%.2f)" % (self.x,self.y,self.z)
    
    def distance(self, p2):
        e = (self.x-p2.x)**2 + (self.y-p2.y)**2 + (self.z-p2.z)**2
        return np.sqrt(e)


class Vector(Point):
    def __init__(self, x=0, y=0, z=0):
        super().__init__(x, y, z)

    def from_np_array(self, arr):
        self.x = arr[0, 0]
        self.y = arr[1, 0]
        self.z = arr[2, 0]

    def from_np_array(self, arr):
        self.x = arr[0, 0]
        self.y = arr[1, 0]
        self.z = arr[2, 0]

    def dot(self, vec2: Point):
        return self.x * vec2.x+self.y * vec2.y+self.z * vec2.z

    def __repr__(self):
        return f"Vector(%.2f,%.2f,%.2f) " % (self.x,self.y,self.z)
    
    def cross(self, vec2: Point):
        x = self.y * vec2.z - self.z * vec2.y
        y = -(self.x * vec2.z - self.z * vec2.x)
        z = self.x * vec2.y - self.y * vec2.x
        return Vector(x, y, z)

    def __iter__(self):
        return iter((self.x, self.y, self.z))
    
    def __add__(self, operand):
        if isinstance(operand, (int, float)):
            return Vector(self.x + operand, self.y + operand, self.z + operand)
        elif isinstance(operand, Vector):
            return Vector(self.x + operand.x, self.y + operand.y, self.z + operand.z)
        elif isinstance(operand, np.ndarray):
            if (operand.shape[1] == 1) and (operand.shape[1] == 3):
                return Vector(self.x+operand[0, 0], self.y+operand[1, 0], self.z+operand[2, 0])
        raise TypeError("Addition does not support this type of operand!")

    def __radd__(self, operand):
        return self + operand

    def __sub__(self, operand):
        if isinstance(operand, (int, float)):
            return Vector(self.x - operand, self.y - operand, self.z - operand)
        elif isinstance(operand, Vector):
            return Vector(self.x - operand.x, self.y - operand.y, self.z - operand.z)
        elif isinstance(operand, np.ndarray):
            if (operand.shape[1] == 1) and (operand.shape[1] == 3):
                return Vector(self.x-operand[0, 0], self.y-operand[1, 0], self.z-operand[2, 0])
        raise TypeError("Addition does not support this type of operand!")

    def __rsub__(self, operand):
        return self - operand

    def __mul__(self, operand):
        if isinstance(operand, (int, float)):
            return Vector(self.x * operand, self.y * operand, self.z * operand)
        elif isinstance(operand, Vector):
            return Vector(self.x * operand.x, self.y * operand.y, self.z * operand.z)
        elif isinstance(operand, np.ndarray):
            if (operand.shape[1] == 1) and (operand.shape[1] == 3):
                return Vector(self.x*operand[0, 0], self.y*operand[1, 0], self.z*operand[2, 0])
        raise TypeError("Addition does not support this type of operand!")

    def __rmul__(self, operand):
        return self * operand


def rotate_vector_quat(quat, vec):
    vec_quat = np.zeros((4, 1))
    vec_quat[1:, 0] = vec
    quat_inv = -quat
    quat_inv[0, 0] = -quat_inv[0, 0]

    return quaternion_mul(quaternion_mul(quat,vec_quat),quat_inv)[1:,0]

def quaternion(theta,vec,deg=True):
    if deg:
        theta = np.radians(theta)
    quat = np.zeros((4,1))
    quat[0,0] = np.cos(theta/2)
    quat[1:,:] = np.sin(theta/2)*vec
    return quat

def quaternion_to_rot_mat(quat)->np.ndarray:
    q_w = quat[0, 0]
    q_v = quat[1:, 0]

    q_vx = np.array([
        [0, -q_v[2], q_v[1]],
        [q_v[2], 0, -q_v[0]],
        [-q_v[1], q_v[0], 0]])

    return (q_w**2 - q_v.T @ q_v)*np.eye(3) + 2 * q_v @ q_v.T + 2*q_w*q_vx


def quaternion_mul(p: np.ndarray, q: np.ndarray):

    p_w = p[0, 0]
    p_v = p[1:, 0]
    q_w = q[0, 0]
    q_v = q[1:, 0]
    p_vx = np.array([
        [0, -p_v[2], p_v[1]],
        [p_v[2], 0, -p_v[0]],
        [-p_v[1], p_v[0], 0]])

    res = np.zeros((4, 1))
    res[0, 0] = p_w*q_w - p_v.T @ q_v
    res[1:, 0] = p_w*q_v + q_w*p_v - p_vx @ q_v

    return res


class Variations(Enum):
    CONSTANT = 1
    LINEAR = 2
    QUADRATIC = 3
    CUBIC = 4
    EXPOTENTIAL = 5


class Model(ABC):
    A: np.ndarray = None
    B: np.ndarray = None
    state: np.ndarray = None
    state_d: np.ndarray = None

    @abstractmethod
    def get_state_space(self):
        pass


class Arm:
    def __init__(self, arm_len=1, origin=(0, 0, 0), arm_width=0.1, arm_thickness=0.1, arm_density=5, arm_density_var=Variations.CONSTANT):
        """
        arm_len : Length of the arm ( in m )
        arm_len : Width of the arm ( in m )
        arm_len : Thickness of the arm ( in m )
        arm_density : Density of the arm matrial (in kg/m)
        arm_density_var : Variation of density of the arm matrial 
        """
        self.arm_len = arm_len
        self.arm_width = arm_width
        self.arm_thickness = arm_thickness
        self.arm_density_var = arm_density_var
        self.conf = Vector(self.arm_len, self.arm_width, self.arm_thickness)
        self.arm_ccp = np.array([[0.5],
                                 [0.5],
                                 [0.0]])
        self.origin = Vector(*origin)
        self.orientation = Vector(0, 0, 0)
        self.arm_mass = self.calculate_mass(arm_density, arm_density_var)
        self.centroid: Vector = self.calculate_centroid()
        Ixx = self.arm_mass*self.arm_len**2
        Iyy = self.arm_mass*self.arm_width**2
        Izz = self.arm_mass*self.arm_thickness**2
        self.inertia = np.array([[Ixx, 0, 0],
                                 [0, Iyy, 0],
                                 [0, 0, Izz]]
                                )
        self.arm = self.origin.to_numpy() + np.array([
            [0, 0, 0],
            [self.arm_len, 0, 0],
            [self.arm_len, self.arm_width, 0],
            [0, self.arm_width, 0],
        ]).T

        self.arm_plots: plt.Line2D = None
        self.plot_initialized = False
        self.ax = False
        self.hinge_point = Vector(0,0,0)

    def get_position_vector(self):
        return self.position

    def set_position_vector(self, position):
        self.position = position

    def get_orientation(self):
        return self.orientation

    def set_orientation(self, orientation):
        self.orientation = orientation

    def calculate_mass(self, arm_density, variation):
        if variation == Variations.CONSTANT:
            return arm_density*self.arm_len

    def calculate_centroid(self) -> Vector:
        if self.arm_density_var == Variations.CONSTANT:
            return self.origin + self.conf*0.5
        else:
            raise "Not implemented!"

    def rotate(self, axis, theta):
        quat = np.array([
            [np.cos(theta)],
            [axis[0]*np.sin(theta)],
            [axis[1]*np.sin(theta)],
            [axis[2]*np.sin(theta)],
        ])

    def plot(self, ax: plt.Axes = None):
        if (not self.ax):
            if not ax:
                raise "Axes is not set!"
            else:
                self.ax = ax
        if not self.plot_initialized:
            self.plot_initialized = True
            x, y, z = self.origin
            self.arm_plots = patches.Polygon(self.arm.T[:,:2], color='orange', fill=True)

            # Add circle to plot
            # self.joint_plot = self.ax.scatter([x],[y], color='black', s=300, label="Joint",zorder=1)
            self.ax.add_patch(self.arm_plots)
        else:
            self.arm_plots.set_xy(self.arm.T[:,:2])

    def set_hinge_point(self, hinge_point=Vector(0, 0, 0)):
        """
        hinge_point : (x,y,z) in percentage of length, width & thickness of arm 
        Eg: (0.5, 0.5, 0.5)
        - 50% of the length 
        - 50% of the width 
        - 50% of the thickness 
        """
        self.hinge_point = self.origin + hinge_point * self.conf

    def get_inertia_about_hinge(self):
        return self.get_inertia_about(self.hinge_point)

    def get_inertia_about(self, axis):
        return (self.arm_mass*self.arm_len**2 / 3) + self.origin.distance(axis)**2 * self.arm_mass

    def get_centroid(self) -> Vector:
        return self.centroid

    def get_mass(self):
        return self.arm_mass

    def update(self,translate,rotate,quaternion = True):
        if quaternion:
            for i in range(4):
                self.arm[:,i] = rotate_vector_quat(rotate,self.arm[:,i])
        else:
            self.arm[:,:] = rotate @ (self.arm + translate)



class Joint(Model):
    def __init__(self, joint_name: str,K: float = 0, b: float = 0.3, g=GRAVITY,radius=0.1,del_t = 0.01):
        """ 
        theta : joint angle(position)
        theta_d : joint velocity
        theta_dd : joint acceleration
        J : inertia of the joint
        b : damping coefficient (friction)
        Tau_g : gravitational torque
        m : mass suspended at the joint end
        r_g : diatacne of the centroid of the mass from the joint
        """
        self.joint_name = joint_name
        self.J = 0
        self.radius = radius
        self.K = K
        self.b = b
        self.theta = 0
        self.theta_d = 0
        self.theta_dd = 0
        self.g = Vector(0, g, 0)
        self.origin = Vector(0,0,0)
        self.arm = None
        self.ax: plt.Axes = None
        self.joint_plot = None
        self.plot_initialized = None
        self.state = np.zeros((2,1))
        self.u = np.zeros((2,1))
        self.del_t = del_t
        self.Tau_g = 0
        self.Tau = 0

    def calculate_state_space_params(self):
        self.A = np.array([[0,    1],
                           [-self.K/self.J,  -self.b/self.J]])
        self.B = np.array([[0,   0],
                           [1/self.J, -1/self.J]])
        # self.C = np.array([[self.del_t,   0],
        #                    [1, self.del_t]])
        # self.D = np.array([[1,0],
        #                    [0,0]])
        self.C = np.eye(2)
        self.D = np.eye(2)

    def init_state(self,theta=0,theta_d=0):
        self.state[0,0] = theta
        self.state[1,0] = theta_d

    def init_control_vec(self,Tau_g,Tau=0):
        self.u[0,0]=Tau
        self.u[1,0]=Tau_g
        
    def get_state_space(self):
        self.state_d = self.A @ self.state + self.B @ self.u
        self.state = self.C @ self.state_d + self.D @ self.state
        
        return self.state_d,self.state

    def set_torque(self, torque):
        self.Tau = torque
        self.u[0,0]=torque

    def set_inertia(self, inertia):
        self.J = inertia

    def get_state_trans_mat(self):
        """
        Return the state transition mat (A)
        """
        return self.A

    def get_input_mat(self):
        """
        Return the input mat (B)
        """
        return self.B

    def get_joint_origin(self) -> np.ndarray:
        self.origin

    def step(self, torque):
        self.get_state_space()

    def set_gravity_torque(self, p: Vector, m: float):
        """
        r_g : Centroid of the arm (in m)
        m : Mass of the arm (in Kg)
        """

        r_g = p - self.origin
        self.Tau_g = (m*self.g).cross(r_g)
        print(self.Tau_g)

    def add_arm(self, arm: Arm, hinge_point: Vector):
        if self.arm:
            raise "Arm is already set!"
        else:
            self.arm = arm

        self.arm.set_hinge_point(hinge_point=hinge_point)
        self.set_inertia(self.arm.get_inertia_about_hinge())
        self.set_gravity_torque(self.arm.get_centroid(), self.arm.get_mass())

    def plot(self, ax):
        if (not self.ax):
            if not ax:
                raise "Axes is not set!"
            else:
                self.ax = ax
        if not self.plot_initialized:
            self.plot_initialized = True
            x, y, z = self.origin
            circle = plt.Circle((0, 0), self.radius, color='b',
                                fill=True, linewidth=2)

            # Add circle to plot
            # self.joint_plot = self.ax.scatter([x],[y], color='black', s=300, label="Joint",zorder=1)
            self.joint_plot = self.ax.add_patch(circle)
        else:
            x, y, z = self.origin
            self.joint_plot.set_data([x], [y])

    def rotate_by(self,theta):
        translate = np.array([
            [0.0],
            [0.0],
            [0],
        ])
        rot_vec = np.array([
            [0],
            [0],
            [1],
        ])
       
        rotate = quaternion(theta,rot_vec)
        # theta = np.radians(10)
        # rotate = np.array([
        #     [np.cos(theta),np.sin(theta),0],
        #     [-np.sin(theta),np.cos(theta),0],
        #     [0,0,1],
        # ])
        self.state[0,0] += theta
        self.arm.update(translate=translate,rotate=rotate)

    def get_state(self):
        return self.state

if __name__ == "__main__":    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')

    from controllers import MPC

    # controller = MPC(model=None)
    # controller.get_control_input()

    arm = Arm(origin=(-0.1, -0.1/2, 0))
    joint = Joint('joint_1',radius=.1)
    joint.add_arm(arm, Vector(0.2, 0.5, 0))
    joint.calculate_state_space_params()
    print(joint.get_state_space())
    joint.plot(ax)
    arm.plot(ax)
    plt.xlim((-1.1, 1.1))
    plt.ylim((-1.1, 1.1))

    sim_steps = 2
    theta_d = 0

    ref_thetas = np.linspace(0,180,sim_steps)
    t = np.linspace(0,10,sim_steps)
    const_rate = (ref_thetas[1]-ref_thetas[0])/(t[1]-t[0])
    ref_theta_ds = np.ones_like(ref_thetas)*const_rate


    for i in range(sim_steps):
        tic = time.time_ns()
        
        [cur_theta,cur_theta_d] = joint.get_state()
        ref_theta,ref_theta_d = ref_thetas[i],ref_theta_ds[i]

        print(f" Current angle: {cur_theta} Current rotation rate: {cur_theta_d}")
        print(f" Ref angle: {ref_theta} Ref rotation rate: {ref_theta_d}\n")

        theta = ref_theta-cur_theta

        # joint.rotate_by(theta)
        print("Before state :",joint.get_state())
        joint.set_torque(1)
        print(joint.get_state_space())
        print("After state :",joint.get_state())
        arm.plot()
        plt.pause(0.1)
        plt.draw()

        # theta_dd,theta_d = joint.get_state_space()
        # toc = time.time_ns()
        # del_t = round((toc-tic)/1e9,3)
        # theta_d_c = np.round(theta/del_t,3)
        # theta_d_a = np.round(theta/0.1,3)
        # print(f"del_t = {del_t} theta_d_c = {theta_d_c} theta_d_a = {theta_d_a}")

    plt.show()