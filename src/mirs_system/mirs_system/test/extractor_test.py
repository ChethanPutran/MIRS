import os
import cv2
import glob
import random
import matplotlib.pyplot as plt
import numpy as np
from mirs_system.ai.vision.estimator.object_identifier import ObjectIdentifier
from mirs_system.ai.vision.estimator.hand_pose import HandPose


TEST_VIDEOS = glob.glob(os.path.join(os.path.join(os.path.dirname(__file__),'videos',"*.mp4")))


def get_random_test_video():
    pth = random.sample(TEST_VIDEOS,1)[0]
    if 'left' in pth:
        pth_right = pth.replace("left","right")
        return (pth,pth_right)
    else:
        pth_left = pth.replace("right","left")
        return (pth_left,pth)

win_titles = ['original_left',
              'original_right',
              'identified_left',
              'identified_right',
              ]

def obj_ientifier_test():
    obj = ObjectIdentifier()

    count = 0

    test_video = get_random_test_video()
    cap_left = cv2.VideoCapture(test_video[0])
    cap_right = cv2.VideoCapture(test_video[1])

    while True:
        print("New img")
        st1,frame_left = cap_left.read()
        st2,frame_right = cap_right.read()

        if (not st1) or (not st2):
            break

        objects_l = obj.identify_object(frame_left)
        objects_r = obj.identify_object(frame_right)

        print(objects_l,objects_r)
        
        count += 4
        cap_left.set(cv2.CAP_PROP_POS_FRAMES, count)
        cap_right.set(cv2.CAP_PROP_POS_FRAMES, count)

    cap_left.release()
    cap_right.release()


    

def extract():

    test_video = get_random_test_video()

    hand_pose_estimator = HandPose()

    video_left = cv2.VideoCapture(test_video[0])
    video_right = cv2.VideoCapture(test_video[1])


    video_frames_count = min(int(video_left.get(cv2.CAP_PROP_FRAME_COUNT)),int(video_right.get(cv2.CAP_PROP_FRAME_COUNT)))
    
    count = 0
    FRAMES_REQUIRED = 30
    SKIP = video_frames_count//FRAMES_REQUIRED


    print(video_frames_count,SKIP)

    hand_poses = []


    while True:
        check1, frame_l = video_left.read()
        check2, frame_r = video_right.read()

        if not(check1) or  not(check2):
            break
        
        if SKIP:
            count+=SKIP

            video_left.set(cv2.CAP_PROP_POS_FRAMES, count)
            video_right.set(cv2.CAP_PROP_POS_FRAMES, count)

        hand_pose = hand_pose_estimator.get_pose_3D(frame_l,frame_r,visualize=True)
        hand_poses.append(hand_pose)

    video_left.release()
    video_right.release()
    

    print("Pose : ",hand_poses)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    #obj_ientifier_test()
    extract()


# ex = Extractor()

# base_dir = os.path.dirname(__file__)
# test_recordings = [
#     os.path.join(base_dir,"left_.avi"),
#     os.path.join(base_dir,"right_.avi"),

# ]

# res = ex.extract(test_recordings)

# print(res)