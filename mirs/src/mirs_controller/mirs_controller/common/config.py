LINK_1_LENGTH = 0.0
LINK_2_LENGTH = 0.0
LINK_3_LENGTH = 0.0
LINK_4_LENGTH = 0.0
LINK_5_LENGTH = 0.0
LINK_6_LENGTH = 0.0

JOINT_NAMES = [
                'SHOULDER_JOINT_1' ,
                'SHOULDER_JOINT_2' ,
                'ELBOW_JOINT',
                'WRIST_JOINT_1' ,
                'WRIST_JOINT_2' ,
                'WRIST_JOINT_3',
                'FINGER_1_JOINT_1',
                'FINGER_1_JOINT_2',
                'FINGER_1_JOINT_3',
                'FINGER_2_JOINT_1',
                'FINGER_2_JOINT_2',
                'FINGER_2_JOINT_3',
                'FINGER_3_JOINT_1',
                'FINGER_3_JOINT_2',
                'FINGER_3_JOINT_3'
                ]

JOINT_SENSOR_NAMES = [
    'SHOULDER_JOINT_1_SENSOR',
                    'SHOULDER_JOINT_2_SENSOR',
                    'ELBOW_JOINT_SENSOR',
                    'WRIST_JOINT_1_SENSOR',
                    'WRIST_JOINT_2_SENSOR',
                    'WRIST_JOINT_3_SENSOR']

JOINTS = { "FINGER_1_JOINT_1":1,
    "FINGER_1_JOINT_2":2,
    'FINGER_1_JOINT_3':3,
    'FINGER_2_JOINT_1':4,
    'FINGER_2_JOINT_2':5,
    'FINGER_2_JOINT_3':6,
    'FINGER_3_JOINT_1' :7,
    'FINGER_3_JOINT_2' :8,
    'FINGER_3_JOINT_3':9,
    'SHOULDER_JOINT_1':10,
    'SHOULDER_JOINT_2' :11,
    'ELBOW_JOINT' :12,
    'WRIST_JOINT_1' :13,
    'WRIST_JOINT_2' :14,
    'WRIST_JOINT_3' :15
}
      
NODES = [
  'NO_NODE',
  'ROBOT',
  'ACCELEROMETER',
  'CAMERA',
  'DISTANCE_SENSOR',
  'GPS',
  'GYRO',
  'INERTIAL_UNIT',
  'MOTOR',
  "SPEAKER",
  "TOUCH_SENSOR",
  "MICROPHONE"
]

JOINT_TYPE = {
    "REVOLUTE" : 0,
    "PRISMATIC":1
}

class MODES:
    MODE_VOICE_ACTIVATED = 0
    MODE_DIRECT = 1

INPUT_EVENTS = {
    'NO_EVENT' : 0,
    'MOUSE_CLICK' : 1,
    'MOUSE_MOVE' : 2,
    'KEYBOARD': 3,
    'JOYSITCK_BUTTON' : 4}


class Error:
    def __init__(self, msg, emergency=False):
        self.EMERGENCY = emergency
        self.MESSAGE = msg

class State:
    STATE_WAITING = 0
    STATE_GRASPING = 1
    STATE_ROTATING = 2
    STATE_RELEASING = 3
    STATE_ROTATING_BACK = 4
