import cv2
import time
import os
import numpy as np
import mediapipe as mp
import math
import matplotlib.pyplot as plt
import uuid

ACTIONS = {
    "pick":[],
    "place":[],
    "move":[],
    "pick":[],
    "pick":[],
}

def get3DHand(image):
    hand = {'inclinations': []}
    return hand


class Hand:
    def __init__(self, fps=15, mode=False, max_hands=1, detection_conf=0.7, track_conf=0.7):
        self.mode = mode
        self.model = None
        self.max_hands = max_hands
        self.detection_conf = detection_conf
        self.track_conf = track_conf
        self.pTime = 0
        self.cTime = 0
        self.fps = fps

        # initialize mediapipe
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
            max_num_hands=1, min_detection_confidence=0.7)

        # For drawing hand skeleton
        self.mpDraw = mp.solutions.drawing_utils

        self.f00 = self.mpHands.HandLandmark.WRIST
        self.f11 = self.mpHands.HandLandmark.THUMB_CMC
        self.f14 = self.mpHands.HandLandmark.THUMB_TIP
        self.f21 = self.mpHands.HandLandmark.INDEX_FINGER_MCP
        self.f24 = self.mpHands.HandLandmark.INDEX_FINGER_TIP
        self.f31 = self.mpHands.HandLandmark.MIDDLE_FINGER_MCP
        self.f34 = self.mpHands.HandLandmark.MIDDLE_FINGER_TIP

    def detect(self, image):
        # Converting BGR image to RGB image for Media pipe processing
        imgRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.raw_img = image
        self.img = image

        # Processing the image
        self.results = self.hands.process(imgRGB)

    def get_landmarks(self, image):
        self.detect(image)
        landmarks = []

        # Getting hand land-marks
        if (self.results.multi_hand_landmarks):

            # Iterating over each hand-landmark
            for hand_landmark in self.results.multi_hand_landmarks:
                for id, land_mark in enumerate(hand_landmark.landmark):
                    print(id)
                    if (id > self.f34):
                        break
                    height, width, _ = image.shape
                    center_x, center_y = int(
                        land_mark.x*width), int(land_mark.y*height)
                    landmarks.append([center_x, center_y])
        return landmarks

    def getFingureActuations(self, img):
        return self.model.predict(img)

    def getFingureStatus(self, landmarks):
        fingure_tips = [4, 8, 12, 16, 20]
        fingures_status = []

        if landmarks:
            for id in fingure_tips:
                # Fingure
                if landmarks[id][2] < landmarks[id-2][2]:
                    fingures_status.append(1)
                else:
                    fingures_status.append(0)
            # Load the gesture recognizer model
            # model = tf.keras.models.load_model('mp_hand_gesture')

            # # Load class names
            # f = open('gesture.names', 'r')
            # classNames = f.read().split('\n')
            # f.close()
            # print(classNames)

            # framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # # Get hand landmark prediction
            # result = hands.process(framergb)

            # className = ''

            # # post process the result
            # if result.multi_hand_landmarks:
            #     landmarks = []
            #     for handslms in result.multi_hand_landmarks:
            #         for lm in handslms.landmark:
            #             # print(id, lm)
            #             lmx = int(lm.x * x)
            #             lmy = int(lm.y * y)

            #             landmarks.append([lmx, lmy])
            #         # Drawing landmarks on frames
            #         mpDraw.draw_landmarks(frame, handslms,
            # mpHands.HAND_CONNECTIONS)
            # # Predict gesture in Hand Gesture Recognition project
            # prediction = model.predict([landmarks])
            # print(prediction)
            # classID = np.argmax(prediction)
            # className = classNames[classID]
            #   # show the prediction on the frame
            # cv2.putText(frame, className, (10, 50), cv2.FONT_HERSHEY_SIMPLEX,1, (0,0,255), 2, cv2.LINE_AA)
            # Initialize the webcam for Hand Gesture Recognition Python project

    def getFingurePositions(self, draw=False):
        landmarks = []

        # Getting hand land-marks
        if (self.results.multi_hand_landmarks):

            # Iterating over each hand-landmark
            for hand_landmark in self.results.multi_hand_landmarks:
                for id, land_mark in enumerate(hand_landmark.landmark):
                    print(id)
                    if (id > self.f34):
                        break
                    height, width, _ = image.shape
                    center_x, center_y = int(
                        land_mark.x*width), int(land_mark.y*height)

                    landmarks.append([center_x, center_y])

                if draw:
                    # Drawing lines connecting the 21 detected points
                    self.mpDraw.draw_landmarks(image, hand_landmark,
                                               self.mpHands.HAND_CONNECTIONS)

        return landmarks, image

    def line(self, landmark1, landmark2):
        # y=c
        if (landmark2[2]-landmark1[2]) == 0:
            return 0, landmark2[2]
        # x=c
        if (landmark2[1]-landmark1[1]) == 0:
            return 90, landmark2[1]

        m = math.degrees((landmark2[2]-landmark1[2]) /
                         (landmark2[1]-landmark1[1]))

        c = (landmark1[2]-landmark1[1])/m
        return m, c

    def getSlope(self, p1, p2):
        if ((p2[0] - p1[0]) == 0):
            print("Zero encountered!")
            return np.inf
        return (p2[1]-p1[1])/(p2[0]-p1[0])

    def getAngleBetweenLines(self, m_l1, m_l2):
        if (m_l1 == np.inf or m_l2 == np.inf):
            if (m_l1 == np.inf and m_l2 == np.inf):
                return 0
            if (m_l1 == np.inf):
                return math.atan(m_l2) - 90
            return math.atan(m_l1) - 90
        if (not (m_l1*m_l2 == -1)):
            angleR = math.atan((m_l2 - m_l1)/(1+(m_l1*m_l2)))
            return math.degrees(angleR)
        return 0

    def getFingureRelativeAngles(self, landmarks,show_in_img = False):

        if (len(landmarks) < 13):
            return

        wrist = landmarks[self.f00]
        thumb = landmarks[self.f11:self.f14+1]
        fingure1 = landmarks[self.f21:self.f24+1]
        fingure2 = landmarks[self.f31:self.f34+1]

        m_l11 = self.getSlope(wrist, thumb[0])
        m_l12 = self.getSlope(thumb[0], thumb[1])
        m_l13 = self.getSlope(thumb[1], thumb[2])
        m_l14 = self.getSlope(thumb[2], thumb[3])
        m_l21 = self.getSlope(wrist, fingure1[0])
        m_l22 = self.getSlope(fingure1[0], fingure1[1])
        m_l23 = self.getSlope(fingure1[1], fingure1[2])
        m_l24 = self.getSlope(fingure1[2], fingure1[3])
        m_l31 = self.getSlope(wrist, fingure2[0])
        m_l32 = self.getSlope(fingure2[0], fingure2[1])
        m_l33 = self.getSlope(fingure2[1], fingure2[2])
        m_l34 = self.getSlope(fingure2[2], fingure2[3])

        #print(m_l11, m_l12, m_l13, m_l14)
        #print(m_l21, m_l22, m_l23, m_l24)
        #print(m_l31, m_l32, m_l33, m_l34)

        theta11 = round(self.getAngleBetweenLines(m_l11, m_l12))
        theta12 = round(self.getAngleBetweenLines(m_l12, m_l13))
        theta13 = round(self.getAngleBetweenLines(m_l13, m_l14))
        theta21 = round(self.getAngleBetweenLines(m_l21, m_l22))
        theta22 = round(self.getAngleBetweenLines(m_l22, m_l23))
        theta23 = round(self.getAngleBetweenLines(m_l23, m_l24))
        theta31 = round(self.getAngleBetweenLines(m_l31, m_l32))
        theta32 = round(self.getAngleBetweenLines(m_l32, m_l33))
        theta33 = round(self.getAngleBetweenLines(m_l33, m_l34))


        if show_in_img:
            cv2.putText(self.img, str(theta11),
                        thumb[0], cv2.FONT_HERSHEY_DUPLEX, .3,  (0, 255, 0), 2)
            cv2.putText(self.img, str(theta12),
                        thumb[1], cv2.FONT_HERSHEY_DUPLEX, .3,  (0, 255, 0), 2)
            cv2.putText(self.img, str(theta13),
                        thumb[2], cv2.FONT_HERSHEY_DUPLEX, .3,  (0, 255, 0), 2)

            cv2.putText(self.img, str(theta21),
                        fingure1[0], cv2.FONT_HERSHEY_DUPLEX, .3,  (255, 255, 0), 2)
            cv2.putText(self.img, str(theta22),
                        fingure1[1], cv2.FONT_HERSHEY_DUPLEX, .3,  (255, 255, 0), 2)
            cv2.putText(self.img, str(theta23),
                        fingure1[2], cv2.FONT_HERSHEY_DUPLEX, .3,  (255, 255, 0), 2)

            cv2.putText(self.img, str(theta31),
                        fingure2[0], cv2.FONT_HERSHEY_DUPLEX, .3,  (0, 255, 255), 2)
            cv2.putText(self.img, str(theta32),
                        fingure2[1], cv2.FONT_HERSHEY_DUPLEX, .3,  (0, 255, 255), 2)
            cv2.putText(self.img, str(theta33),
                        fingure2[2], cv2.FONT_HERSHEY_DUPLEX, .3,  (0, 255, 255), 2)
        
        
        return [[theta11,theta12,theta13],[theta21,theta22,theta23],[theta31,theta32,theta33]]
    
    def getFingureRelativeAngles3D(self, landmarks_3D):

        if (len(landmarks_3D) < 13):
            return

        wrist = landmarks_3D[self.f00]
        thumb = landmarks_3D[self.f11:self.f14+1]
        fingure1 = landmarks_3D[self.f21:self.f24+1]
        fingure2 = landmarks_3D[self.f31:self.f34+1]

        m_l11 = self.getSlope(wrist, thumb[0])
        m_l12 = self.getSlope(thumb[0], thumb[1])
        m_l13 = self.getSlope(thumb[1], thumb[2])
        m_l14 = self.getSlope(thumb[2], thumb[3])
        m_l21 = self.getSlope(wrist, fingure1[0])
        m_l22 = self.getSlope(fingure1[0], fingure1[1])
        m_l23 = self.getSlope(fingure1[1], fingure1[2])
        m_l24 = self.getSlope(fingure1[2], fingure1[3])
        m_l31 = self.getSlope(wrist, fingure2[0])
        m_l32 = self.getSlope(fingure2[0], fingure2[1])
        m_l33 = self.getSlope(fingure2[1], fingure2[2])
        m_l34 = self.getSlope(fingure2[2], fingure2[3])

        print(m_l11, m_l12, m_l13, m_l14)
        print(m_l21, m_l22, m_l23, m_l24)
        print(m_l31, m_l32, m_l33, m_l34)

        theta11 = round(self.getAngleBetweenLines(m_l11, m_l12))
        theta12 = round(self.getAngleBetweenLines(m_l12, m_l13))
        theta13 = round(self.getAngleBetweenLines(m_l13, m_l14))
        theta21 = round(self.getAngleBetweenLines(m_l21, m_l22))
        theta22 = round(self.getAngleBetweenLines(m_l22, m_l23))
        theta23 = round(self.getAngleBetweenLines(m_l23, m_l24))
        theta31 = round(self.getAngleBetweenLines(m_l31, m_l32))
        theta32 = round(self.getAngleBetweenLines(m_l32, m_l33))
        theta33 = round(self.getAngleBetweenLines(m_l33, m_l34))

        return [[theta11,theta12,theta13],[theta21,theta22,theta23],[theta31,theta32,theta33]]
    
    def get_angles_from_image(self,image):
        landmarks_2D = self.get_landmarks(image)
        theta = self.getFingureRelativeAngles(landmarks=landmarks_2D)
        return theta



class Task:
    def __init__(self):
        self.task_object = ''
        self.X_ee = np.zeros((1,3))
        self.X_obj = np.zeros((1,3))
        self.theta = np.zeros((2,3))
        self.id = str(uuid.uuid1())
        self.trajectory = []

    def set_object_pos(self,X):
        self.X_obj[:]=X

    def set_ee_pos(self,X):
        self.X_ee[:] = X

    def set_object(self,obj):
        self.task_object = obj

    def set_hand_theta(self,theta):
        self.theta[:] = theta

   
    def interpolate_theta(self,theta_i,theta_f,n_steps = 10,time_ = 3,accl=1,plot=False):
        """
        time_stamp = 3s
        speed = 20mm/s

        """
        qg = theta_f
        qs = theta_i
        tg = time_
        delT = time_/float(n_steps) 
        tb = 0.5 * tg - (0.5*np.sqrt(((accl*tg**2)-4*(qg-qs))/(accl))) # blend time
        tp = tg - tb                                               # tg-tb



        # #=============================================== Time ===============================================#
        tx = np.arange(0,tb,delT)        # 0 <= t <= tb          -> 1st segment position 0 to tb
        ty = np.arange(tb+delT,tp,delT)   # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        tz = np.arange(tp,tg ,delT)      # (tg - tb) <= t <= tg  -> 2nd segment position tb to tg-tb
        t = np.concatenate((tx,ty,tz))



        # #=============================================== Displacements ===============================================#
        d1 = qs + 0.5*accl*(tx**2)                         # 0 <= t <= tb          -> 1st segment position 0 to tb
        d2 = qs + (0.5*accl*(tb**2)) + (accl*tb*(ty-tb))   # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        d3 = qg - (0.5*accl*((tg-tz)**2))                  # (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg
        d = np.concatenate((d1,d2,d3))


        # #=============================================== Velocities ===============================================#

        v1 = accl*tx                         # 0 <= t <= tb          -> 1st segment position 0 to tb
        v2 = accl*tb*np.ones(ty.shape[0])          # tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
        v3 = accl*(tg-tz)                    # (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg
        v = np.concatenate((v1,v2,v3))

        # #=============================================== Accelerations ===============================================#

        a1 = accl*np.ones(tx.shape[0])              #1st segment acceleration 0 to tb
        a2 = np.zeros_like(ty)                           #2nd segment acceleration tb to tg-tb
        a3 = -accl*np.ones(tz.shape[0])             #3rd segment acceleration tg-tb to tg
        a = np.concatenate((a1,a2,a3))


        if plot:

            fig, ax = plt.subplots(1,3)
            fig.set_figwidth(20)
            fig.set_figheight(5)

            #Plot of Position vs Time 
            ax[0].plot(t,d)
            ax[0].set_title("Plot of Position vs Time ")
            ax[0].set_ylabel('Position, q(s) (in m)')
            ax[0].set_xlabel('Time, t (in s)')

            #Plot of Velocity vs Time
            ax[1].plot(t,v)
            ax[1].set_title("Plot of Velocity vs Time")
            ax[1].set_ylabel('Velocity (in m/s)')
            ax[1].set_xlabel('Time (in s)')

            #Plot of Acceleration vs Time
            ax[2].plot(t,a)
            ax[2].set_title("Plot of Acceleration vs Time")
            ax[2].set_ylabel('Acceleration (in m/s**2)')
            ax[2].set_xlabel('Time (in s)')

            plt.savefig("Plots",dpi=150)
            plt.show()

        return [d,v,a,t]

    def get_trajectory(self):

        ### Check here for proper ravel 
        theta_i = np.array(self.hand.get_angles_from_image(self.__X_i.img)).ravel()
        theta_f = np.array(self.hand.get_angles_from_image(self.__X_f.img)).ravel()
        
        self.trajectory = self.interpolate_theta(theta_i,theta_f,self.n_steps,time_=self.__time_gap,)

        return self.trajectory
