import matplotlib.pyplot as plt
import cv2 as cv
import mediapipe as mp
import numpy as np
from mirs_system.ai.vision.calibration.calibration import Calibration
from .utils import DLT


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
        self.P0 , self.P1 = Calibration.get_stereo_projection_matrix()

    def extract_3D_keypoints(self, frame_l, frame_r):
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
