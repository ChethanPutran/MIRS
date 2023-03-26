import numpy as np
import cv2
import time
import os

DATSET_FOLDER = 'C:\\Chethan\\Mechanical\\projects\\Major_Project\\software\\System\\mirs\\datasets'

CLASSES = ['close', 'open', 'pick', 'pitch', 'yaw', 'roll', 'place']

DATA_TIME = 10  # s
NUM_CLASS_ITEMS = 25
start_l = 0
start_i = 0


def collector():
    F_PATH = "C:\\Chethan\\Mechanical\\projects\\Major_Project\\software\\System\\mirs\\utils\\history.txt"
    with open(F_PATH, 'r') as file:
        data = file.readline().split(" ")
        start_i = int(data[1])
        start_l = int(data[0])

    for label in range(start_l, len(CLASSES)):
        print(
            f"Enter {NUM_CLASS_ITEMS-start_i} samples for '{CLASSES[label]}' operation \n")
        for i in range(start_i, NUM_CLASS_ITEMS):
            try:
                a = str(input("Press enter to start capture."))
                print(i+1, "\n")
                if not (a == ""):
                    raise Exception('Error')

                file_name = f"{DATSET_FOLDER}\\{label}_{i}.avi"

                video = cv2.VideoCapture(0)

                if (video.isOpened() == False):
                    print("Error reading video file")

                frame_width = int(video.get(3))
                frame_height = int(video.get(4))
                size = (frame_width, frame_height)

                result = cv2.VideoWriter(file_name,
                                         cv2.VideoWriter_fourcc(*'MJPG'),
                                         10, size)
                # cv2.setWindowProperty('image', cv2.WINDOW_GUI_NORMAL, 1)
                end_time = time.time() + DATA_TIME
                i = 0

                while (True):
                    ret, frame = video.read()
                    print(i)
                    i += 1
                    cv2.waitKey(1)
                    if ret == True:
                        result.write(frame)
                        cv2.imshow('Frame', frame)
                        if time.time() > end_time:
                            break
                    else:
                        break

                video.release()
                result.release()
                cv2.destroyAllWindows()
                print("Done.")

            except KeyboardInterrupt as e:
                with open(F_PATH, 'w') as file:
                    file.write(f"{label} {i}")
                exit(0)

        start_i = 0


if __name__ == '__main__':
    collector()
