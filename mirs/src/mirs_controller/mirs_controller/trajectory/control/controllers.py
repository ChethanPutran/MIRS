import numpy as np
import time 
from mirs_controller.dynamics.dynamics import Dynamics
from mirs_interfaces.hardware.hardware import Hardware

class ArmJointController:
    def __init__(self,joints,Kp,Kd,Ki,tolerance=0.01):
        self.joints = joints
        self.n = len(joints)
        self.pre_theta = np.zeros((self.n,1))
        self.theta = np.zeros((self.n,1))
        self.pre_theta_dot = np.zeros((self.n,1))
        self.theta_dot = np.zeros((self.n,1))
        self.dynamics  = Dynamics()
        self.Kd = Kd
        self.Ki = Ki
        self.Kp = Kp
        self.tolerance = tolerance

    
    def calulate_control_input(self,desired_theta,desired_theta_dot,desired_acc):

        theta,theta_dot =  Hardware.get_feedback()

        # Calculating errors
        d_theta = desired_theta - theta 
        d_theta_dot = desired_theta_dot - theta_dot 

        tau_ = desired_acc + self.Kp*d_theta + self.Kd*d_theta_dot

        alpha = self.dynamics.M(theta)

        # beta =   C + G + F
        beta =  self.dynamics.C(theta,theta_dot) +  self.dynamics.G(theta) + self.dynamics.F(theta)

        tau = tau_*alpha + beta

        return tau


    # Closed loop control
    def forward(self,desired_theta,desired_theta_dot,desired_acc):
        # u =  self.calulate_control_input(desired_theta,desired_theta_dot,desired_acc)
        # if(u<self.tolerance):
        #     return True
        
        # Hardware.set_torque(u)
        Hardware.set_motor(desired_theta,desired_theta_dot)
        res = Hardware.get_feedback()
        return res


    # set the PID control parameters
    def set_control_pid(self, p, i, d):  
        self.Kp = p
        self.Kd = d
        self.Ki = i

class EEController:
    def __init__(self,motor,Kp,Kd):
        self.motor = motor
        self.pre_theta = 0
        self.pre_theta_dot = 0
        self.Kp = Kp
        self.Kd = Kd 
        self.dynamics  = Dynamics()

    def forward(self,desired_theta,desired_theta_dot,desired_acc):
        theta = self.feedback_sensor.get_position()
        theta_dot = self.feedback_sensor.get_velocity()

        # Calculating errors
        d_theta = desired_theta - theta 
        d_theta_dot = desired_theta_dot - theta_dot 

        tau_ = desired_acc + self.Kp*d_theta + self.Kd*d_theta_dot

        alpha = self.dynamics.D(theta)

        # beta = M + C + F
        beta = self.dynamics.M(theta,theta_dot) + self.dynamics.C(theta,theta_dot) + self.dynamics.F(theta,theta_dot)

        tau = tau_*alpha + beta

        return tau


