from system.hardware.connection import Connection


class Motor:
    def __init__(self, motor_num=0):
        self.id = motor_num
    # Setters

    def set_position(self, position):  # rad or meters
        Connection.set_data(data_label='m1', value=position)
        pass

    def set_acceleration(self, acceleration):          # rad/s^2 or m/s^2
        pass

    def set_velocity(self, velocity):                  # rad/s or m/s
        pass

    def set_force(self, force):                        # N
        pass

    def set_torque(self, torque):                      # N*m
        pass

    def set_available_force(self, force):              # N
        pass

    def set_available_torque(self, torque):            # N*m
        pass

    def set_control_pid(self, p, i, d):  # set the PID control parameters
        pass

    def enable_force_feedback(self,  sampling_period):
        pass

    # Getters
    def disable_force_feedback(self):
        pass

    def get_force_feedback_sampling_period(self):
        pass

    def get_force_feedback(self):
        pass

    def enable_torque_feedback(self,  sampling_period):
        pass

    def disable_torque_feedback(self):
        pass

    def get_torque_feedback_sampling_period(self):
        pass

    def get_torque_feedback(self):
        pass

    def get_type(self):
        pass

    def get_target_position(self):
        pass

    def get_min_position(self):
        pass

    def get_max_position(self):
        pass

    def get_velocity(self):
        pass

    def get_max_velocity(self):
        pass

    def get_acceleration(self):
        pass

    def get_available_force(self):
        pass

    def get_max_force(self):
        pass

    def get_available_torque(self):
        pass

    def get_max_torque(self):
        pass

    def get_multiplier(self):
        pass

    def get_brake(self):
        pass

    def get_position_sensor(self):
        pass


class Motors:
    MOTOR_FINGER_1_JOINT_1 = Motor(0)
    MOTOR_FINGER_1_JOINT_2 = Motor(1)
    MOTOR_FINGER_1_JOINT_3 = Motor(2)
    MOTOR_FINGER_2_JOINT_1 = Motor(3)
    MOTOR_FINGER_2_JOINT_2 = Motor(4)
    MOTOR_FINGER_2_JOINT_3 = Motor(5)
    MOTOR_FINGER_3_JOINT_1 = Motor(6)
    MOTOR_FINGER_3_JOINT_2 = Motor(7)
    MOTOR_FINGER_3_JOINT_3 = Motor(8)
    MOTOR_SHOULDER_JOINT_1 = Motor(9)
    MOTOR_SHOULDER_JOINT_2 = Motor(10)
    MOTOR_ELBOW_JOINT = Motor(11)
    MOTOR_WRIST_JOINT_1 = Motor(12)
    MOTOR_WRIST_JOINT_2 = Motor(13)
    MOTOR_WRIST_JOINT_3 = Motor(14)
