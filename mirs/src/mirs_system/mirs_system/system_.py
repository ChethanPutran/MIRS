# from system.robot.sensors import DistanceSensor, PositionSensor
# from system.robot.robot import Robot
# from system.robot.motor import Motor
# from system.ai.voice.voice import Voice
# from system.ai.vision.camera import Camera
# from system.ai.vision.hand import Hand
# from system.ai.vision.coordinate_estimator import CoordinateEstimator
from system.ai.vision.object_identifier import ObjectIdentifier
# from system.events.events import Event, Process
import cv2
import time
import os
import numpy as np


class Task:
    def __init__(self, object, x, y, z, motor_vals):
        self.__OBJECT = object
        self.__X = x
        self.__Y = y
        self.__Z = z
        self.__HAND_ACTUATIONS = motor_vals

    def get_x(self):
        return self.__X

    def get_y(self):
        return self.__Y

    def get_z(self):
        return self.__Z

    def get_object(self):
        return self.__OBJECT

    def get_actuations(self):
        return self.__HAND_ACTUATIONS

    def get_position(self):
        return (self.__X, self.__Y, self.__Z)


class TaskExtractor:
    def __init__(self):
        self.sucess = True
        self.error = None
        self.task_queue = []
        # self.hand = Hand()
        # self.coordinate_estimator = CoordinateEstimator()
        # self.object_identifier = ObjectIdentifier()

    def make_coordinates(self, landmarks, depths):
        return np.hstack((landmarks, depths))

    def extract(self, recording):

        video = cv2.VideoCapture(recording)
        while True:
            try:
                check, frame = video.read()

                if check:
                    print("Recording processed sucessfully.")
                    break

                img_l, img_r = frame

                # Read each frame from the webcam
                landmarks = self.hand.get_landmarks(img_l)

                self.coordinate_estimator.compute_depth(img_l, img_r)

                depths = self.coordinate_estimator.get_way_points_depth(
                    img_l, img_r, landmarks)
                landmarks_3D = self.make_coordinates(landmarks, depths)

                angles = self.hand.getFingureRelativeAngles3D(landmarks_3D)

                object, location, confidence = self.object_identifier.identify(
                    img_l, ['pencil', 'pen', 'calculator']) or self.object_identifier.identify(
                    img_r, ['pencil', 'pen', 'calculator']) or 'No object'

                z = self.coordinate_estimator.get_point_depth(
                    img_l, img_r, (location[1], location[0]))

                self.task_queue.append(
                    Task(object, location[0], location[1], z, angles))

            except Exception as e:
                self.error = e
                self.sucess = False
                break

        video.release()
        return self.task_queue, self.sucess, self.error


class TaskExecutor:
    def __init__(self):
        self.sucess = True
        self.error = None
        self.task_queue = []
        # self.hand = Hand()
        # self.coordinate_estimator = CoordinateEstimator()
        self.object_identifier = ObjectIdentifier()

    def make_coordinates(self, landmarks, depths):
        return np.hstack((landmarks, depths))

    def extract(self, recording):

        video = cv2.VideoCapture(recording)
        while True:
            try:
                check, frame = video.read()

                if check:
                    print("Recording processed sucessfully.")
                    break

                # img_l, img_r = frame
                img_l, img_r = frame

                # # Read each frame from the webcam
                # landmarks = self.hand.get_landmarks(img_l)

                # self.coordinate_estimator.compute_depth(img_l, img_r)

                # depths = self.coordinate_estimator.get_way_points_depth(
                #     img_l, img_r, landmarks)
                # landmarks_3D = self.make_coordinates(landmarks, depths)

                # angles = self.hand.getFingureRelativeAngles3D(landmarks_3D)

                # object, location, confidence = self.object_identifier.identify(img_l, ['pencil', 'pen', 'calculator'])

                # z = self.coordinate_estimator.get_point_depth(
                #     img_l, img_r, (location[1], location[0]))

                # self.task_queue.append(
                #     Task(object, location[0], location[1], z, angles))

            except Exception as e:
                self.error = e
                self.sucess = False
                break

        video.release()
        return self.task_queue, self.sucess, self.error


class TaskRecorder:
    def __init__(self, camera_index=0, fps=5):
        self.RECORDING = None
        self.RECORDING_STATUS = True
        self.RECORDING_ERROR = None
        self.CAMERA_INDEX = camera_index
        self.FPS = fps

    def record(self):
        cap = cv2.VideoCapture(self.CAMERA_INDEX)
        frame_width = int(cap.get(3))
        frame_height = int(cap.get(4))
        SIZE = (frame_width, frame_height)

        file_name = os.path.join(os.path.dirname(os.path.realpath(
            __file__)), f"recordings\\recording_{str(int(time.time()))}.avi")

        print(f"Recording file name : {file_name}")

        recording = cv2.VideoWriter(
            file_name,
            cv2.VideoWriter_fourcc(*'XVID'),
            self.FPS,
            SIZE)

        count = 0
        MAX_FRAMES = 100

        # while (not (Event.CUR_EVENT == Event.EVENT_END_RECORD)):
        while True:
            status, frame = cap.read()

            if (cv2.waitKey(1) == ord('q')) or (count > MAX_FRAMES):
                break

            recording.write(frame)
            cv2.imshow('Frame', frame)

            count += 1

            print(count)
            time.sleep(.5)

        print(f"Saving recording...")

        cap.release()
        recording.release()
        cv2.destroyAllWindows()
        self.RECORDING = file_name

        # Event.fire(Event.EVENT_END_RECORD)

        return self.RECORDING_STATUS, self.RECORDING_ERROR

    def get_recording(self):
        return self.RECORDING


class System:
    def __init__(self):
        self.task = None
        # self.voice = Voice()
        self.task_recorder = TaskRecorder()
        self.task_executor = TaskExtractor()
        self.task_extractor = TaskExecutor()
        self.recording = []
        self.tasks = []

    def start(self):
        # while not (Event.CUR_EVENT == Event.EVENT_EXIT):
        cmd = ''
        while not (cmd == 'exit'):
            # cmd = self.voice.listen()
            cmd = input("Enter your command : ")
            print('\n')
            if cmd == 'record':
                # self.voice.speak("Press 'Q' to stop recording.")
                # Event.fire(Event.EVENT_START_RECORD)
                print("Recording...")
                success, error = self.task_recorder.record()
                if success:
                    print("Recording sucessfull")
                    self.recording = self.task_recorder.get_recording()
                else:
                    print("Recording Failed!")
                    print("Error %s" % error)
            if cmd == 'process':
                if len(self.recording) > 0:
                    success, error = self.task_extractor.extract(
                        self.recording)
                    if not success:
                        print(
                            f"Error! Couldn't extract task!!! Reason : {error}")
                    else:
                        print("Recording processed sucessfully.")

            # elif cmd == 'execute':
            #     #Event.fire(Event.EVENT_EXECUTE_TASK)
            #     print("Executing...")
            #     # self.execute()
            #     self.robot.set_tasks(tasks)
            #     sucess, error = self.robot.execute()
            #     if (not sucess):
            #         raise Exception(f"Emergency Stop : Reason ({error})")
            #     print("Task completed sucessfully :)")

            elif cmd == 'exit':
                # Event.fire(Event.EVENT_EXIT)
                print("Exiting...")
                break

        # self.voice.speak("Exiting... Bye...")
        print("Exiting... Bye...")


if __name__ == "__main__":
    sys = System()
    sys.start()
