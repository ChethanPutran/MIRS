"""
Access camera of the robot
Get image one by one
Used to find the co-ordinate (x,y,z) of a point from the image
"""

import cv2
import numpy as np
#from system.events.events import Event
from datetime import datetime
import glob


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


def PointData(x, y, z, obj):
    return {'x': x, 'y': y, 'z': z, 'object': obj}


class Camera:

    """ 
    ***** Calibration params (f, b, u0, v0) *****

    f - focal length
    b - base line
    u0 - x offset in image coordinate
    v0 - y offset in image coordinate

    """
    f = 10  # in mm
    b = 10
    u0 = 2
    v0 = 1

    def __init__(self, size=100):
        self.camera = 0
        self.size = size

    def cannyImage(self, image):
        # 1.Conveting coloured image to grayscale image
        grayScaleImage = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # 2.Reducing noise and smoothening image
        bluredGSImage = cv2.GaussianBlur(grayScaleImage, (5, 5), 0)

        # Determing the edges based on gradient(taking derivative(f(x,y)) x->along width of the message y-> along height)
        # (image,low_threshold,hight_threshold)
        canny = cv2.Canny(bluredGSImage, 50, 150)

        return canny
