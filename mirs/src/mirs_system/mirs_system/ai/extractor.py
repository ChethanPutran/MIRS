import os
import cv2
import numpy as np
# from mirs_system.ai.task.task import Task
# from mirs_system.ai.vision.estimator.action_identifier import ActionIdentifier
# from mirs_system.ai.vision.estimator.object_identifier import ObjectIdentifier
# from mirs_system.ai.vision.estimator.object_pose import PoseEstimator
from mirs_system.ai.vision.estimator.depth_estimator import CoordinateEstimator
# from ai.vision.estimator.hand_pose import HandPose


SINGLE_OBJECT = True


# class Extractor:
#     def __init__(self):
#         self.task_queue = []
#         # self.action_identifier = ActionIdentifier()
#         # self.object_pose_identifier = PoseEstimator()
#         # self.object_identifier = ObjectIdentifier()
#         # self.hand_pose_estimator = HandPose()
#         self.coordinate_estimator = CoordinateEstimator()
#         # self.recognition_object = {
#         #     'id':0,
#         #     'position' : [],
#         #     'orientation': [4],
#         #     'size' : [2],
#         #     'position_on_image':[],
#         #     'size_on_image':[],
#         #     'number_of_colors':0,
#         #     'colors':[],
#         #     'model' : ''
#         #     }

#     def make_coordinates(self, landmarks, depths):
#         return np.hstack((landmarks, depths))

#     def extract(self, recordings):

#         video_left = cv2.VideoCapture(recordings[0])
#         video_right = cv2.VideoCapture(recordings[1])

#         frame_group = []
#         frame_group_size = 15
#         group_size = 0

#         task_object = None
#         found_object = False
#         object_frame = []

#         while True:
#             try:
#                 if SINGLE_OBJECT:
#                     check1, frame_left = video_left.read()
#                     check2, frame_right = video_right.read()

#                     if check1 and check2:
#                         print("Recording processed sucessfully.")
#                         break

#                     if (group_size <= frame_group_size):
#                         group_size += 1
#                         frame_group.append((frame_left, frame_right,))
#                         continue

#                     # Frame group full (extract task)
#                     # action = self.action_identifier.identify_action(
#                     #     frame_group)
#                     action = ''

#                     start_frame_l, start_frame_r = frame_group[0]
#                     end_frame_l, end_frame_r = frame_group[0]

#                     # Assume only single object
#                     if not found_object:
#                         task_object, pos_i, _ = self.object_identifier.identify_object(
#                             start_frame_l)
#                         object_frame = self.object_identifier.get_object_frame()
#                         pos_f = self.object_identifier.get_object_position(
#                             start_frame_l, object_frame)
#                         found_object = True

#                     else:
#                         pos_i = self.object_identifier.get_object_position(
#                             start_frame_l, object_frame)
#                         pos_f = self.object_identifier.get_object_position(
#                             start_frame_l, object_frame)

#                     obj_initial_pose = self.object_pose_identifier.get_pose(
#                         task_object, pos_i, start_frame_l, start_frame_l)
#                     obj_final_pose = self.object_pose_identifier.get_pose(
#                         task_object, pos_f, end_frame_l, end_frame_r)

#                     initial_position = self.coordinate_estimator.get_3Dpoint_depth(
#                         frame_left, frame_right, pos_i)
#                     final_position = self.coordinate_estimator.get_3Dpoint_depth(
#                         frame_left, frame_right, pos_f)

#                     hand_3d_points = self.hand_pose_estimator.extract_3D_keypoints_from_frame(
#                         frame_left, frame_right)

#                     self.task_queue.append(Task(task_object,
#                                                 action,
#                                                 initial_position,
#                                                 final_position,
#                                                 obj_initial_pose,
#                                                 obj_final_pose,
#                                                 hand_landmarks=hand_3d_points

#                                                 ))

#                     # Resetting group size
#                     group_size = 0

#             except Exception as e:
#                 raise (False, [], e)

#         video_left.release()
#         video_right.release()

#         return (True, self.task_queue, None)




def main():
    from mirs_system.ai.vision.calibration.raspberrypi import Raspberrypi

    # te = Extractor()

    # test_folder = os.path.dirname(__file__)

    # rasp = Raspberrypi()
    #files,err = rasp.get_sample_video(test_folder)
    # files,err = rasp.get_sample_image(test_folder)

    
    # print(files)

    # if err:
    #     print("Error :".join(err))
    #     return

    files = ('/home/chethan/Desktop/MIRS/mirs/src/mirs_system/mirs_system/ai/left_sample_image.jpg', 
             '/home/chethan/Desktop/MIRS/mirs/src/mirs_system/mirs_system/ai/right_sample_image.jpg')

    left_img,right_img = files

    ce = CoordinateEstimator()
    ce.estimate(left_img,right_img)
    

print(__name__)

if __name__ == "__main__":
    main()