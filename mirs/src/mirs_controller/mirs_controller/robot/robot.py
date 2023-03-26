from system.robot.dynamics import Dynamics
from system.robot.kinematics import Kinematics
# from system.robot.motion_planning import Planner
from system.robot.motor import Motors
import numpy as np
from system.events.events import Event
import time


class Joint:
    def __init__(self):
        self.type = 0
        self.ROTATIONAL = 0
        self.LINEAR = 1


class MODES:
    MODE_VOICE_ACTIVATED = 0
    MODE_DIRECT = 1


class INPUT_EVENTS:
    NO_EVENT = 0
    MOUSE_CLICK = 1
    MOUSE_MOVE = 2
    KEYBOARD = 3
    JOYSITCK_BUTTON = 4


class Hand:
    MOTORS = [
        Motors.MOTOR_FINGER_1_JOINT_1,
        Motors.MOTOR_FINGER_1_JOINT_2,
        Motors.MOTOR_FINGER_1_JOINT_3,
        Motors.MOTOR_FINGER_2_JOINT_1,
        Motors.MOTOR_FINGER_2_JOINT_2,
        Motors.MOTOR_FINGER_2_JOINT_3,
        Motors.MOTOR_FINGER_3_JOINT_1,
        Motors.MOTOR_FINGER_3_JOINT_2,
        Motors.MOTOR_FINGER_3_JOINT_3,
    ]

    @staticmethod
    def perform(angles, action=None):
        #  if (counter <= 0):
        #     if state == State['WAITING']:
        #         if (distance_sensor.get_value() < 500):
        #         state = 'GRASPING'
        #         counter = 8
        #         print("Grasping can\n")
        #         for motor in hand_motors:
        #             motor.set_position(0.85)

        #     elif state == State['GRASPING']:
        #         for motor in arm_motors:
        #             motor.set_position(target_positions)
        #         print("Rotating arm\n")
        #         state = 'ROTATING'

        #     elif state == State['ROTATING']:
        #         if (position_sensor.get_value() < -2.3):
        #         counter = 8
        #         print("Releasing can\n")
        #         state = 'RELEASING'
        #         for motor in hand_motors:
        #             motor.set_position(motor.get_min_position())

        #     elif state == State['RELEASING']:
        #         for motor in arm_motors:
        #         motor.set_position(0.0)
        #         printf = ("Rotating arm back\n")
        #         state = 'ROTATING_BACK'

        #     elif state == State['ROTATING_BACK']:
        #         if (position_sensor.get_value() > -0.1):
        #         state = 'WAITING'
        #         printf("Waiting can\n")
        #     counter -= 1

        # print("Task finished.")
        # robot.shut_down()
        sucess, error = False, Error('Error')
        return sucess, error


class Error:
    def __init__(self, msg, emergency=False):
        self.EMERGENCY = emergency
        self.MESSAGE = msg


class Arm:
    MOTORS = [
        Motors.MOTOR_WRIST_JOINT_1,
        Motors.MOTOR_WRIST_JOINT_2,
        Motors.MOTOR_WRIST_JOINT_3,
        Motors.MOTOR_ELBOW_JOINT,
        Motors.MOTOR_SHOULDER_JOINT_1,
        Motors.MOTOR_SHOULDER_JOINT_2
    ]


class State:
    STATE_WAITING = 0
    STATE_GRASPING = 1
    STATE_ROTATING = 2
    STATE_RELEASING = 3
    STATE_ROTATING_BACK = 4


class Robot:
    HAND = Hand()
    ARM = Arm()
    MODES = MODES
    INPUT_EVENTS = INPUT_EVENTS
    TIME_STEP = 0.1  # in sec
    STATES = State()
    TASK_QUEUE = []
    EXECUTION_ERROR = None
    EXECUTION_STATUS = None
    TIME_STEP = 0.1  # in s
    SLEEP_TIME = 0.1  # in s
    SPEED = 20  # mm/s
    CUR_STATE = (0, 0, 0)

    @staticmethod
    def shut_down():
        Event.fire(Event.EVENT_EXIT)

    @staticmethod
    def set_tasks(tasks):
        for task in tasks:
            Robot.TASK_QUEUE.append(task)

    @staticmethod
    def execute():
        while (len(Robot.TASK_QUEUE) > 0):
            cur_task = Robot.TASK_QUEUE.pop()
            if (not (Event.CUR_EVENT == Event.EVENT_PAUSE)):
                sucess, error = Robot.perform(cur_task)
                if not sucess:
                    print(error)
                    if error.EMERGENCY:
                        Event.fire(Event.EVENT_PAUSE)
                    else:
                        Event.fire(Event.EVENT_EXIT)
                        Robot.EXECUTION_STATUS = False
                        Robot.EXECUTION_ERROR = error
                        break
            else:
                print('.', end='')
                time.sleep(Robot.SLEEP_TIME)
        return Robot.EXECUTION_STATUS, Robot.EXECUTION_ERROR

    @staticmethod
    def perform(task) -> tuple[bool, Error]:
        for motor in Robot.ARM.MOTORS:
            motor.set_velocity(Robot.SPEED)

        # Compute inverse kinematics

        # Compute trajectory

        # sucess, error = Robot.move(task.position())
        # Implement move
        sucess, error = Robot.HAND.perform(
            task.get_actuations(), task.get_object())

        return sucess, error

    @staticmethod
    def move(trajctory):
        # Implement PID
        sucess, error = None, None
        return sucess, error

    def get_current_state(self):
        return Robot.CUR_STATE

    def get_hand_motors(self, pos):
        return Robot.HAND.MOTORS[pos]

    def get_arm_motors(self, pos):
        return Robot.ARM.MOTORS[pos]

    def step(self, duration):
        self.TIME_STEP += duration

    # milliseconds
    def wait_for_user_input_event(self, WbUserInputEvent, event_type, timeout):
        pass

    def cleanup(self):
        pass

    def get_time(self):
        pass

    def get_urdf(self, prefix):
        pass

    def get_name(self):
        pass

    def get_model(self):
        pass

    def get_custom_data(self):
        pass

    def set_custom_data(self, data):
        pass

    def get_mode(self):
        pass

    def set_mode(self,  mode, arg):
        pass

    def get_synchronization(self):
        pass

    def get_supervisor(self):
        pass

    def get_project_path(self):
        pass

    def get_world_path(self):
        pass

    def get_basic_time_step(self):
        pass

    def get_device(self, name):
        pass

    def get_number_of_devices(self):
        pass

    def get_device_by_index(self, index):
        pass

    def get_type(self):
        pass

    def battery_sensor_enable(self, sampling_period):
        pass

    def battery_sensor_disable(self):
        pass

    def battery_sensor_get_sampling_period(self):
        pass

    def battery_sensor_get_value(self):
        pass

    def task_new(self,  task,   *param):  # create a task
        pass

    def mutex_new(self):
        pass

    def mutex_lock(self, WbMutexRef):
        pass

    def mutex_unlock(self, WbMutexRef):
        pass

    def mutex_delete(self, WbMutexRef):
        pass

    def pin_to_static_environment(self, pin):
        pass

    def get_controller_name(self):
        pass

    def get_data(self):
        pass

    def set_data(self, data):
        pass
