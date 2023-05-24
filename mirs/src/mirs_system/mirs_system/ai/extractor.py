import os
import cv2
import numpy as np
from mirs_system.ai.task.task import Task
from mirs_system.ai.vision.estimator.models.action_identifier.model import ActionIdentifier
from mirs_system.ai.vision.estimator.object_identifier import ObjectIdentifier
from mirs_system.ai.vision.estimator.depth_estimator import DepthEstimator
from mirs_system.ai.vision.camera.parameters import FOV_H,FOV_V,T_0_cam
# from mirs_system.ai.vision.estimator.object_pose import PoseEstimator
from mirs_system.ai.vision.estimator.hand_pose import HandPose


SINGLE_OBJECT = True


class Extractor:
    def __init__(self):
        self.task_queue = []
        
        self.action_identifier = ActionIdentifier()
        # self.object_pose_identifier = PoseEstimator()
        self.object_identifier = ObjectIdentifier()
        self.hand_pose_estimator = HandPose()
        self.depth_estimator = DepthEstimator()
        # self.recognition_object = {
        #     'id':0,
        #     'position' : [],
        #     'orientation': [4],
        #     'size' : [2],
        #     'position_on_image':[],
        #     'size_on_image':[],
        #     'number_of_colors':0,
        #     'colors':[],
        #     'model' : ''
        #     }

    def make_coordinates(self, landmarks, depths):
        return np.hstack((landmarks, depths))

    def extract(self, recordings):

        video_left = cv2.VideoCapture(recordings[0])
        video_right = cv2.VideoCapture(recordings[1])

        task_queue = [] 

        while True:
            check1, frame_left = video_left.read()
            check2, frame_right = video_right.read()

            if not(check1) or  not(check2):
                print("Recording processed sucessfully.")
                break

            task = Task()
            

            # Object identification

            # 1. Identify object & it's position in pixel coordinate
            object_l = self.object_identifier.identify_object(frame_left)
            object_r = self.object_identifier.identify_object(frame_right)

            task.set_object(object_l['class_id'])

            # 2. Identify depth of the point and get 3D coordinate
            x_pixel,y_pixel,z = self.depth_estimator.get_point_depth(frame_left,frame_right,object_l)

            # 2. Trasform pixel coordinate to camera-coordinate
            x_cam,y_cam,z_cam = x_pixel*FOV_H,y_pixel*FOV_V,z

            X_cam = np.array([
                [x_cam],
                [y_cam],
                [z_cam],
                [1]
                ])

            # 3. Get the object position w.r.to the Robot
            X_0 = T_0_cam @ X_cam
            
            
            # 4. Save task object position
            task.set_object_pos(X_0)

            
            # Compute object pose
            # obj_initial_pose = self.object_pose_identifier.get_pose(
            #     task_object, pos_i, start_frame_l, start_frame_l)
            # obj_final_pose = self.object_pose_identifier.get_pose(
            #     task_object, pos_f, end_frame_l, end_frame_r)

            
            # Hand motion identification

            # 1. Get hand pose from image
            #theta , (x_ee_pixel,y_ee_pixel,z_ee_pixel) = self.hand_pose_estimator.get_pose_3D(frame_left,frame_right)
            theta , (x_ee_pixel,y_ee_pixel) = self.hand_pose_estimator.get_pose(frame_left)

            # 2. Scale hand pose to the meet the hand requirements
            task.set_hand_theta(theta)

            # 3. Get hand pam start position and consider this as e.e position
            x_ee_cam,y_ee_cam = x_ee_pixel*FOV_H,y_ee_pixel*FOV_V

            # Compute this
            z_ee = 1

            X_ee_cam = np.array([
                [x_ee_cam],
                [y_ee_cam],
                [z_ee],
                [1]
                ])

            # 3. Get the object position w.r.to the Robot
            X_ee_0 = T_0_cam @ X_ee_cam


            # 4. Save hand starting position as E.E positon
            task.set_ee_pos(X_ee_0)

            task_queue.append(task)

        video_left.release()
        video_right.release()

        # Map the hand-ee position to robot-ee frame
        task_theta = []
        K = 0.5  # Scaling paramter between real ee and human hand
        robot_ee = task_theta * K

        return task_queue

