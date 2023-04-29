import matplotlib.pyplot as plt
import cv2 as cv
import mediapipe as mp
import numpy as np
from utils import DLT, get_projection_matrix, write_keypoints_to_disk


class HandPose:
    def __init__(self):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.frame_shape = (640, 480)
        # create hand keypoints detector object.
        self.hand_l = self.mp_hands.Hands(min_detection_confidence=0.5,
                                          max_num_hands=1, min_tracking_confidence=0.5)
        self.hand_r = self.mp_hands.Hands(min_detection_confidence=0.5,
                                          max_num_hands=1, min_tracking_confidence=0.5)

        # projection matrices
        self.P0 = get_projection_matrix(0)
        self.P1 = get_projection_matrix(1)

    def extract_3D_keypoints_from_frame(self, frame_l, frame_r):
        # crop to 720x720.
        # Note: camera calibration parameters are set to this resolution.If you change this, make sure to also change camera intrinsic parameters
        if frame_l.shape[1] != 720:
            frame_l = frame_l[:, self.frame_shape[1]//2 - self.frame_shape[0] //
                              2:self.frame_shape[1]//2 + self.frame_shape[0]//2]
            frame_r = frame_r[:, self.frame_shape[1]//2 - self.frame_shape[0] //
                              2:self.frame_shape[1]//2 + self.frame_shape[0]//2]

        # the BGR image to RGB.
        frame_l = cv.cvtColor(frame_l, cv.COLOR_BGR2RGB)
        frame_r = cv.cvtColor(frame_r, cv.COLOR_BGR2RGB)

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.

        frame_l.flags.writeable = False
        frame_r.flags.writeable = False

        results0 = self.hand_l.process(frame_l)
        results1 = self.hand_r.process(frame_r)

        # prepare list of hand keypoints of this frame
        # frame0 kpts
        frame_l_keypoints = []
        if results0.multi_hand_landmarks:
            for hand_landmarks in results0.multi_hand_landmarks:
                for p in range(21):
                    # print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                    pxl_x = int(
                        round(frame_l.shape[1]*hand_landmarks.landmark[p].x))
                    pxl_y = int(
                        round(frame_l.shape[0]*hand_landmarks.landmark[p].y))
                    kpts = [pxl_x, pxl_y]
                    frame_l_keypoints.append(kpts)

        # no keypoints found in frame:
        else:
            # if no keypoints are found, simply fill the frame data with [-1,-1] for each kpt
            frame_l_keypoints = [[-1, -1]]*21

        # frame_r kpts
        frame_r_keypoints = []
        if results1.multi_hand_landmarks:
            for hand_landmarks in results1.multi_hand_landmarks:
                for p in range(21):
                    # print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                    pxl_x = int(
                        round(frame_r.shape[1]*hand_landmarks.landmark[p].x))
                    pxl_y = int(
                        round(frame_r.shape[0]*hand_landmarks.landmark[p].y))
                    kpts = [pxl_x, pxl_y]
                    frame_r_keypoints.append(kpts)

        else:
            # if no keypoints are found, simply fill the frame data with [-1,-1] for each kpt
            frame_r_keypoints = [[-1, -1]]*21

        # calculate 3d position
        frame_p3ds = []
        for uv1, uv2 in zip(frame_l_keypoints, frame_r_keypoints):
            if uv1[0] == -1 or uv2[0] == -1:
                _p3d = [-1, -1, -1]
            else:
                # calculate 3d position of keypoint
                _p3d = DLT(self.P0, self.P1, uv1, uv2)
            frame_p3ds.append(_p3d)

        frame_p3ds = np.array(frame_p3ds).reshape((21, 3))
        return frame_p3ds

    def extract_3D_keypoints_from_stream(self, input_stream1, input_stream2):
        # input video stream
        cap0 = cv.VideoCapture(input_stream1)
        cap1 = cv.VideoCapture(input_stream2)
        caps = [cap0, cap1]

        # set camera resolution if using webcam to 1280x720. Any bigger will cause some lag for hand detection
        for cap in caps:
            cap.set(3, self.frame_shape[1])
            cap.set(4, self.frame_shape[0])

        # create hand keypoints detector object.
        hands0 = self.mp_hands.Hands(min_detection_confidence=0.5,
                                     max_num_hands=1, min_tracking_confidence=0.5)
        hands1 = self.mp_hands.Hands(min_detection_confidence=0.5,
                                     max_num_hands=1, min_tracking_confidence=0.5)

        # containers for detected keypoints for each camera
        kpts_cam0 = []
        kpts_cam1 = []
        kpts_3d = []

        while True:

            # read frames from stream
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            if not ret0 or not ret1:
                break

            # crop to 720x720.
            # Note: camera calibration parameters are set to this resolution.If you change this, make sure to also change camera intrinsic parameters
            if frame0.shape[1] != 720:
                frame0 = frame0[:, self.frame_shape[1]//2 - self.frame_shape[0] //
                                2:self.frame_shape[1]//2 + self.frame_shape[0]//2]
                frame1 = frame1[:, self.frame_shape[1]//2 - self.frame_shape[0] //
                                2:self.frame_shape[1]//2 + self.frame_shape[0]//2]

            # the BGR image to RGB.
            frame0 = cv.cvtColor(frame0, cv.COLOR_BGR2RGB)
            frame1 = cv.cvtColor(frame1, cv.COLOR_BGR2RGB)

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            frame0.flags.writeable = False
            frame1.flags.writeable = False
            results0 = hands0.process(frame0)
            results1 = hands1.process(frame1)

            # prepare list of hand keypoints of this frame
            # frame0 kpts
            frame0_keypoints = []
            if results0.multi_hand_landmarks:
                for hand_landmarks in results0.multi_hand_landmarks:
                    for p in range(21):
                        # print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                        pxl_x = int(
                            round(frame0.shape[1]*hand_landmarks.landmark[p].x))
                        pxl_y = int(
                            round(frame0.shape[0]*hand_landmarks.landmark[p].y))
                        kpts = [pxl_x, pxl_y]
                        frame0_keypoints.append(kpts)

            # no keypoints found in frame:
            else:
                # if no keypoints are found, simply fill the frame data with [-1,-1] for each kpt
                frame0_keypoints = [[-1, -1]]*21

            kpts_cam0.append(frame0_keypoints)

            # frame1 kpts
            frame1_keypoints = []
            if results1.multi_hand_landmarks:
                for hand_landmarks in results1.multi_hand_landmarks:
                    for p in range(21):
                        # print(p, ':', hand_landmarks.landmark[p].x, hand_landmarks.landmark[p].y)
                        pxl_x = int(
                            round(frame1.shape[1]*hand_landmarks.landmark[p].x))
                        pxl_y = int(
                            round(frame1.shape[0]*hand_landmarks.landmark[p].y))
                        kpts = [pxl_x, pxl_y]
                        frame1_keypoints.append(kpts)

            else:
                # if no keypoints are found, simply fill the frame data with [-1,-1] for each kpt
                frame1_keypoints = [[-1, -1]]*21

            # update keypoints container
            kpts_cam1.append(frame1_keypoints)

            # calculate 3d position
            frame_p3ds = []
            for uv1, uv2 in zip(frame0_keypoints, frame1_keypoints):
                if uv1[0] == -1 or uv2[0] == -1:
                    _p3d = [-1, -1, -1]
                else:
                    # calculate 3d position of keypoint
                    _p3d = DLT(self.P0, self.P1, uv1, uv2)
                frame_p3ds.append(_p3d)

            '''
            This contains the 3d position of each keypoint in current frame.
            For real time application, this is what you want.
            '''
            frame_p3ds = np.array(frame_p3ds).reshape((21, 3))
            kpts_3d.append(frame_p3ds)

            # Draw the hand annotations on the image.
            frame0.flags.writeable = True
            frame1.flags.writeable = True
            frame0 = cv.cvtColor(frame0, cv.COLOR_RGB2BGR)
            frame1 = cv.cvtColor(frame1, cv.COLOR_RGB2BGR)

            if results0.multi_hand_landmarks:
                for hand_landmarks in results0.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        frame0, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

            if results1.multi_hand_landmarks:
                for hand_landmarks in results1.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        frame1, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            cv.imshow('cam1', frame1)
            cv.imshow('cam0', frame0)

            k = cv.waitKey(1)
            if k & 0xFF == 27:
                break  # 27 is ESC key.

        cv.destroyAllWindows()
        for cap in caps:
            cap.release()

        return np.array(kpts_cam0), np.array(kpts_cam1), np.array(kpts_3d)

    def run_3D_hand_recognizer(self):

        input_stream1 = 'media/cam0_test.mp4'
        input_stream2 = 'media/cam1_test.mp4'

        kpts_cam0, kpts_cam1, kpts_3d = self.extract_3D_keypoints_from_stream(
            input_stream1, input_stream2, self.P0, self.P1)

        # this will create keypoints file in current working folder
        write_keypoints_to_disk('kpts_cam0.dat', kpts_cam0)
        write_keypoints_to_disk('kpts_cam1.dat', kpts_cam1)
        write_keypoints_to_disk('kpts_3d.dat', kpts_3d)

    def visualize_3d(p3ds):
        """Apply coordinate rotations to point z axis as up"""
        Rz = np.array(([[0., -1., 0.],
                        [1.,  0., 0.],
                        [0.,  0., 1.]]))
        Rx = np.array(([[1.,  0.,  0.],
                        [0., -1.,  0.],
                        [0.,  0., -1.]]))

        p3ds_rotated = []
        for frame in p3ds:
            frame_kpts_rotated = []
            for kpt in frame:
                kpt_rotated = Rz @ Rx @ kpt
                frame_kpts_rotated.append(kpt_rotated)
            p3ds_rotated.append(frame_kpts_rotated)

        """this contains 3d points of each frame"""
        p3ds_rotated = np.array(p3ds_rotated)

        """Now visualize in 3D"""
        thumb_f = [[0, 1], [1, 2], [2, 3], [3, 4]]
        index_f = [[0, 5], [5, 6], [6, 7], [7, 8]]
        middle_f = [[0, 9], [9, 10], [10, 11], [11, 12]]
        ring_f = [[0, 13], [13, 14], [14, 15], [15, 16]]
        pinkie_f = [[0, 17], [17, 18], [18, 19], [19, 20]]
        fingers = [pinkie_f, ring_f, middle_f, index_f, thumb_f]
        fingers_colors = ['red', 'blue', 'green', 'black', 'orange']

        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for i, kpts3d in enumerate(p3ds_rotated):
            if i % 2 == 0:
                continue  # skip every 2nd frame
            for finger, finger_color in zip(fingers, fingers_colors):
                for _c in finger:
                    ax.plot(xs=[kpts3d[_c[0], 0], kpts3d[_c[1], 0]], ys=[kpts3d[_c[0], 1], kpts3d[_c[1], 1]], zs=[
                            kpts3d[_c[0], 2], kpts3d[_c[1], 2]], linewidth=4, c=finger_color)

            # draw axes
            ax.plot(xs=[0, 5], ys=[0, 0], zs=[0, 0], linewidth=2, color='red')
            ax.plot(xs=[0, 0], ys=[0, 5], zs=[0, 0], linewidth=2, color='blue')
            ax.plot(xs=[0, 0], ys=[0, 0], zs=[0, 5],
                    linewidth=2, color='black')

            # ax.set_axis_off()
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_zticks([])

            ax.set_xlim3d(-7, 8)
            ax.set_xlabel('x')
            ax.set_ylim3d(-7, 8)
            ax.set_ylabel('y')
            ax.set_zlim3d(0, 15)
            ax.set_zlabel('z')
            ax.elev = 0.2*i
            ax.azim = 0.2*i
            plt.savefig('figs/fig_' + str(i) + '.png')
            plt.pause(0.01)
            ax.cla()
