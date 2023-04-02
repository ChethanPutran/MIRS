import numpy as np
from dynamics.dynamics import Dynamics

class JointController:
    def __init__(self,name,motor,Kp,Kd,Ki):
        self.motor = motor
        self.pre_theta = 0
        self.pre_theta_dot = 0
        self.Kp = Kp
        self.Kd = Kd 
        self.Ki = Ki
        self.dynamics  = Dynamics()
        self.name = name

    def calulate_control_input(self,desired_theta,desired_theta_dot,desired_acc):
        theta = self.motor.get_position()
        theta_dot = self.motor.get_velocity()

        # Calculating errors
        d_theta = desired_theta - theta 
        d_theta_dot = desired_theta_dot - theta_dot 

        tau_ = desired_acc + self.Kp*d_theta + self.Kd*d_theta_dot

        alpha = self.dynamics.D(theta)

        # beta = M + C + F
        beta = self.dynamics.M(theta,theta_dot) + self.dynamics.C(theta,theta_dot) + self.dynamics.F(theta,theta_dot)

        tau = tau_*alpha + beta

        return tau

    def forward(self,desired_theta,desired_theta_dot,desired_acc):
        u =  self.calulate_control_input(desired_theta,desired_theta_dot,desired_acc)
        self.motor.set_torque(u)


class EEController:
    def __init__(self,motor,Kp,Kd):
        self.motor = motor
        self.pre_theta = 0
        self.pre_theta_dot = 0
        self.Kp = Kp
        self.Kd = Kd 
        self.dynamics  = Dynamics()

    def forward(self,desired_theta,desired_theta_dot,desired_acc):
        theta = self.motor.get_position()
        theta_dot = self.motor.get_velocity()

        # Calculating errors
        d_theta = desired_theta - theta 
        d_theta_dot = desired_theta_dot - theta_dot 

        tau_ = desired_acc + self.Kp*d_theta + self.Kd*d_theta_dot

        alpha = self.dynamics.D(theta)

        # beta = M + C + F
        beta = self.dynamics.M(theta,theta_dot) + self.dynamics.C(theta,theta_dot) + self.dynamics.F(theta,theta_dot)

        tau = tau_*alpha + beta

        return tau


