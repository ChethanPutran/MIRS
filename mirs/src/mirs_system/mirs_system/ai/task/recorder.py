import cv2
import time
import os
import numpy as np

class Recorder:
    def __init__(self, camera_index=0, fps=5):
        self.RECORDING = None
        self.RECORDING_STATUS = True
        self.RECORDING_ERROR = None
        self.CAMERA_INDEX = camera_index
        self.FPS = fps
        self.stop = False

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
        while self.stop:
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

    def stop_record(self):
        self.stop = True
    def get_recording(self):
        return self.RECORDING
