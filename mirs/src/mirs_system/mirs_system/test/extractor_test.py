import os
import cv2
import glob
import matplotlib.pyplot as plt
from mirs_system.ai.vision.estimator.object_identifier import ObjectIdentifier
from mirs_system.ai.vision.estimator.models.action_identifier.model import ActionIdentifierModel


test_video_folder = os.path.join(os.path.dirname(__file__),'videos')

TEST_VIDEO = [os.path.join(test_video_folder,"box_left.avi"),os.path.join(test_video_folder,"box_right.avi")]


win_titles = ['original_left',
              'original_right',
              'identified_left',
              'identified_right',
              ]

def obj_ientifier_test():
    obj = ObjectIdentifier()

    count = 0

    cap_left = cv2.VideoCapture(TEST_VIDEO[0])
    cap_right = cv2.VideoCapture(TEST_VIDEO[1])

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

def action_identifier_test():
    action_identifier = ActionIdentifierModel()

    count = 0

    cap_left = cv2.VideoCapture(TEST_VIDEO[0])


    frame_group = []
    group_size = 0
    count = 0
    SKIP = 4

    
    video_frames_count = int(cap_left.get(cv2.CAP_PROP_FRAME_COUNT))

    while True:
        check1, frame = cap_left.read()

        count += SKIP
        cap_left.set(cv2.CAP_PROP_POS_FRAMES, count)

        if not check1:
            print("Recording processed sucessfully.")
            break

        if (group_size < action_identifier.SEQ_LEN):
            group_size += 1
            frame_group.append(frame)
            continue

  
        group_size = 0

        for frame in frame_group:
            cv2.imshow("Video",frame)

            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        # Frame group full (extract task)
        action = action_identifier.predict(frame_group)
        
        print("Identified Action : ",action)

        # CLear frame group
        frame_group.clear()


    cap_left.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    #obj_ientifier_test()
    action_identifier_test()


# ex = Extractor()

# base_dir = os.path.dirname(__file__)
# test_recordings = [
#     os.path.join(base_dir,"left_.avi"),
#     os.path.join(base_dir,"right_.avi"),

# ]

# res = ex.extract(test_recordings)

# print(res)