import math
import numpy as np


class Planner:
    def __init__(self, acceleration, start_position, goal_position, goal_time, mean_position, delT=0.0001):
        self.delaT = delT
        self.mean_position = (start_position+goal_position) / \
            2          # Mean Position
        self.mean_time = goal_time / 2                                   # Mean Time
        # Constant Acceleration
        self.const_acceleration = acceleration
        self.blend_time = 0.5*goal_time-0.5 *\
            math.sqrt(((acceleration*goal_time**2)
                       - 4*(goal_position-start_position))/(acceleration))           # Blend time
        self.tg_tb = goal_time - self.blend_time
        self.blend_distance = mean_position\
            - acceleration*self.blend_time * \
            (self.mean_time-self.blend_time)  # Blend position
        # 0 <= t <= tb          -> 1st segment position 0 to tb
        self.times1 = np.linspace(0, self.blend_time, int(1/delT))
        # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        self.times2 = np.linspace(self.blend_time, self.tg_tb, int(1/delT))
        # (tg - tb) <= t <= tg  -> 3rd segment position  tg-tb to tg
        self.times3 = np.linspace(self.tg_tb, goal_time, int(1/delT))
        self.times = np.concatenate((self.times1, self.times2, self.times3))

    def plan(self):
        pass

    def generate(self):
        pass


class Robot:
    name = ''
    n_joints = 6
    n_links = 6

    home_config = ''

    def show():
        pass


class JointSpaceMotionModel():
    pass


class UpdateErrorDynamicsFromStep():
    pass


def load_robot():
    return Robot()


def ode45():
    pass


robot = load_robot()
n_joints = robot.n_joints


# Set up simulation parameters
t_span = np.arange(0, 5, 0.01)

# Joint angles
q0 = np.zeros((n_joints, 1))
q0[2] = np.pi/4      # Something off center


# Joint velocities
qd0 = np.zeros((n_joints, 1))


# Combine joint angles and velocities to form state
initial_state = np.vstack((q0, qd0))

# Set up joint control targets
target_joint_position = [np.pi/2, np.pi/3,
                         np.pi/6, 2*np.pi/3 - np.pi/2 - np.pi/3]
target_joint_velocity = np.zeros((n_joints, 1))
target_joint_acceleration = np.zeros((n_joints, 1))


# Display robot position
robot.show(target_joint_position)


computed_torque_motion = JointSpaceMotionModel(
    "RigidBodyTree", robot, "MotionType", "ComputedTorqueControl")

UpdateErrorDynamicsFromStep(computed_torque_motion, 0.2, 0.1)

q_DesComputedTorque = [target_joint_position,
                       target_joint_velocity,
                       target_joint_acceleration]


pdMotion = JointSpaceMotionModel(
    "RigidBodyTree", robot, "MotionType", "PDControl")
pdMotion.Kp = np.diag(300*np.ones(1, 6))
pdMotion.Kd = np.diag(10*np.ones(1, 6))


qDesPD = [target_joint_position,
          target_joint_velocity]

[tComputedTorque, yComputedTorque] = ode45(@(t, y)derivative(computedTorqueMotion, y, qDesComputedTorque), tSpan, initialState)
[tIndepJoint, yIndepJoint] = ode45(@(t, y)derivative(IndepJointMotion, y, qDesIndepJoint), tSpan, initialState)
[tPD, yPD] = ode15s(@(t, y)derivative(pdMotion, y, qDesPD), tSpan, initialState)
