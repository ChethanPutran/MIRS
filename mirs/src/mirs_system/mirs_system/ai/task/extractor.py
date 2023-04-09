import cv2
import numpy as np
from .task import Task
from ai.vision.estimator.action_identifier import ActionIdentifier
from ai.vision.estimator.object_identifier import ObjectIdentifier
from ai.vision.estimator.object_pose import PoseEstimator
from ai.vision.estimator.depth_estimator import CoordinateEstimator

class Detector:
    def __init__(self):
        self.load_cascade()

    """ Loads the Pen-Cascade """

    def regioOfInterest(self, image):
        height = image.shape[0]  # No. of rows = height
        # Creating polygon arround the lane
        polygons = np.array([(200, height), (1100, height), (550, 250)])

        # Creating a black image having same size as that of of test image
        mask = np.zeros_like(image)

        # Filling back image with generated triangle
        cv2.fillPoly(mask, polygons, 255)

        # Taking bitwise and between image and mask to get the are of interest
        maskedImage = cv2.bitwise_and(image, mask)
        return maskedImage

    def load_cascade(self):
        # Loading face cascades for face detection
        self.cascade = cv2.CascadeClassifier("\\cascade\\cascade.xml")
        pass

    """ Detect the Pen in the given image -> Returns x,y,w,h of the rectangle around the detected pen """

    def detect_object(self, image, same=True):

        # Converting the colored image to gray scale image
        grayScaleImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detecting the pen and it returns the dimensions of the pen(x-start,y-start,width,height)
        pen = self.cascade.detectMultiScale(
            grayScaleImage, scaleFactor=1.1, minNeighbors=5)

        # print(pen)

        # Getting the dimensions of first detected pen
        x, y, w, h = pen[0]

        # return x,y,w,h

        # Drawing the rectangle around the pen
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 3)

        # Resizing the image
        resizedImg = cv2.resize(
            image, (int(image.shape[1]/3), int(image.shape[0]/3)))

        # Dispalying the image
        cv2.imshow("Image", resizedImg)

        if (same):
            # Waiting for key to be pressed
            cv2.waitKey(0)

            # Destroying the window
            cv2.destroyAllWindows()

    def detect_object_pos(self, image, object):
        return 1

    def drawRectangle(self, frame, contours):
        global movement
        contourArea = 100

        for contour in contours:

            if cv2.contourArea(contour) < contourArea:

                continue

            movement = 1
            # Getting the dimensions of the contour
            (x, y, w, h) = cv2.boundingRect(contour)

            # Drawing rectangle around the contour having thickness 3px
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)

        return frame, movement

    def cannyImage(self, image):
        # 1.Conveting coloured image to grayscale image
        grayScaleImage = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # 2.Reducing noise and smoothening image
        bluredGSImage = cv2.GaussianBlur(grayScaleImage, (21, 21), 0)

        # #Determing the edges based on gradient(taking derivative(f(x,y)) x->along width of the message y-> along height)
        # canny = cv2.Canny(bluredGSImage,50,150)#(image,low_threshold,hight_threshold)

        return bluredGSImage

    def movement_identifier(self, first_frame, current_frame):
        firstFrame = None
        contourArea = 8000
        fileName = "motion.csv"
        movements = [None, None]
        motionTimes = []
        # Initializing movement to 0
        movement = 0

        firstFrame = self.cannyImage(first_frame)
        currentFrame = self.cannyImage(current_frame)

        # Getting difference between first frame and current frame
        deltaFrame = cv2.absdiff(firstFrame, currentFrame)

        # Getting threshold frame
        thresholdFrame = cv2.threshold(
            deltaFrame, 30, 255, cv2.THRESH_BINARY)[1]

        # Removing noise
        thresholdFrame = cv2.dilate(thresholdFrame, None, iterations=4)

        # Getting contours
        (contours, _) = cv2.findContours(thresholdFrame.copy(),
                                         cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Drawing rectangle arround the contour if it has desired area
        frameWithRectangle, movement = self.drawRectangle(
            currentFrame, contours)

        # movements.append(movement)
        # movements = movements[-3:]

        # if movements[-1] == 1 and movements[-2] == 0 :
        #     motionTimes.append(datetime.now())

        # if movements[-1] == 0 and movements[-2] == 1 :
        #     motionTimes.append(datetime.now())

        # # Dispalying the image
        # cv2.imshow("Real", frameWithRectangle)
        # # cv2.imshow("Image", deltaFrame)
        # # cv2.imshow("Threshold", thresholdFrame)

        # try:
        #     for i in range(0,len( motionTimes),2):
        #         df = df.append({"Entered Time": motionTimes[i],"Left Time": motionTimes[i+1]},ignore_index=True)

        #     #Checking if file exits or not,if the file exits then the index is continued from privious results
        #     if (os.path.isfile(fileName)):

        #         df.to_csv("temp.csv")

        #         with open(fileName,'r') as destination:

        #             lines = destination.readlines()
        #             lastLine = lines[-1]

        #         lastIndex = lastLine.split(',')[0]

        #         with open(fileName,'a') as destination:

        #             with open("temp.csv",'r') as source:

        #                 lines = source.readlines()

        #                 for i in range(1,len(lines)):
        #                     items = lines[i].split(',')
        #                     items[0] = int(lastIndex)+ (i+1)
        #                     line = ','.join(map(str,items))
        #                     destination.write(line)

        #         os.remove("temp.csv")
        #     else:
        #         df.to_csv(fileName)
        # except Exception as e:
        #     print(e)
        #     print("Could not create file")

        # plot = Plot(fileName)
        # plot.makePlot()

class Extractor:
    def __init__(self):
        self.sucess = True
        self.error = None
        self.task_queue = []
        self.action_identifier = ActionIdentifier()
        self.object_pose_identifier = PoseEstimator()
        self.object_identifier = ObjectIdentifier()
        self.recognition_object = {
            'id':0, 
            'position' : [],
            'orientation': [4],
            'size' : [2],
            'position_on_image':[],
            'size_on_image':[],
            'number_of_colors':0,
            'colors':[],
            'model' : ''
            }

    def make_coordinates(self, landmarks, depths):
        return np.hstack((landmarks, depths))

    def extract(self, recordings):

        video_left = cv2.VideoCapture(recordings[0])
        video_right = cv2.VideoCapture(recordings[1])


        frame_group = []
        frame_group_size = 15

        while True:
            try:
                check1, frame_left = video_left.read()
                check2, frame_right = video_right.read()

                if check1 and check2:
                    print("Recording processed sucessfully.")
                    break

                if(len(frame_group)<=frame_group_size):
                    frame_group.append((frame_left,frame_right))
                    continue


                # Frame group full (extract task)
                action = self.action_identifier.identify(frame_group)
                task_object, pos, confidence= self.object_identifier.identify_object(frame_group[0],["cube","cylinder","sphere"])

                [center_x, center_y, x, y, w, h] = pos
                obj_initial_pos = self.object_pose_identifier.get_pose(task_object,pos,*frame_group[0])
                obj_final_pos = self.object_pose_identifier.get_pose(task_object,pos,*frame_group[-1])
                self.task_queue.append(Task(task_object,action, obj_initial_pos,obj_final_pos))

            except Exception as e :
                self.error = e
                self.sucess = False
                raise (self.sucess,self.error)

        video_left.release()
        video_right.release()

        return self.sucess, self.task_queue, self.error
