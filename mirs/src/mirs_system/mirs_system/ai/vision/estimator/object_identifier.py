#from mediapipe import solutions
import cv2
import numpy as np
import os
from matplotlib.image import imread
import matplotlib.pyplot as plt
from .models.yolov8 import labels,yolov8
from ultralytics.yolo.utils.plotting import Annotator


class ObjectIdentifier:
    def __init__(self, MIN_CONF=0.4):
        self.MIN_CONF = MIN_CONF
        self.LABELS = labels.LABELS
        self.DIR_NAME = os.path.dirname(os.path.realpath(__file__))
        self.model = yolov8.Model()
        self.TEST_IMAGE_FILE_NAME = os.path.join(self.DIR_NAME, "test","test_object.jpg")
        self.COLORS = np.random.randint(
            0, 255, size=(len(self.LABELS), 3), dtype='uint8')

    def identify_object(self, img):
        objects = self.identify_objects(img)

        for obj in objects:
            if obj['conf'] > self.MIN_CONF:
                return obj

    def identify_objects(self, img,draw_box=False):
        results = self.model.predict(img)

        objects = []

        for result in results:
            for box in result.boxes:
                class_id = result.names[box.cls[0].item()]
                cords = box.xyxy[0].tolist()
                cords = [(round(cords[0]),round(cords[1]),),(round(cords[2]),round(cords[3]),)]

                conf = round(box.conf[0].item(), 2)
                objects.append({'class_id':class_id,'cords':cords,'conf':conf})
             
        if draw_box:
            img = self.draw_box(img,results)
            return objects,img

        return objects

    def draw_box(self, img,results):
        annotator = Annotator(img)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                annotator.box_label(b, self.model.names[int(c)])
          
        return annotator.result() 

    def get_object_position(self,frame,object_frame,show=False):
        img_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        template =  cv2.cvtColor(object_frame,cv2.COLOR_BGR2GRAY)
        
        height,width = frame.shape[:2]
        result = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)

        #Finding the indices of the object 
        min_val,max_val,min_loc,max_loc = cv2.minMaxLoc(result)
        
        obj_position = (max_loc[0],max_loc[1],template.shape[1],template.shape[0])

        if show:
            plt.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            plt.imshow(cv2.cvtColor(object_frame,cv2.COLOR_BGR2RGB))
            plt.imshow(result)
            
            top_left = max_loc
            bottom_right = (max_loc[0]+template.shape[1],max_loc[1]+template.shape[0])

            cv2.rectangle(frame,top_left,bottom_right,(0,0,255),5)
            plt.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))  
            plt.show()

        return obj_position
       

    def get_object_frame(self,frame,object_location):
        x, y, w, h = object_location[2], object_location[3], object_location[4],  object_location[5]
        crop =  frame[y:h,x:w]
        return crop

