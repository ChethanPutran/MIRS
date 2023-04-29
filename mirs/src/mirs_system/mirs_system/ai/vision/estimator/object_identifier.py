#from mediapipe import solutions
import cv2
import numpy as np
import os
from matplotlib.image import imread
import matplotlib.pyplot as plt
from .models.yolov7 import labels,yolov7


class ObjectIdentifier:
    def __init__(self, MIN_CONF=0.5):
        self.MIN_CONF = MIN_CONF
        self.LABELS = labels.LABELS
        self.DIR_NAME = os.path.dirname(os.path.realpath(__file__))
        self.model = yolov7.Model()
        self.TEST_IMAGE_FILE_NAME = os.path.join(self.DIR_NAME, "test","test_object.jpg")
        self.COLORS = np.random.randint(
            0, 255, size=(len(self.LABELS), 3), dtype='uint8')

    def identify_object(self, img):
        outputs = self.model.predict(img)

        IMG_WIDTH = img.shape[1]
        IMG_HEIGHT = img.shape[0]

        for output in outputs:
            for detection in output:
                # Getting scores
                scores = detection[5:]

                # Finding max score
                class_id = np.argmax(scores)

                # Getting the corresponding confidencce
                confidence = scores[class_id]

                # Create box if the confidence is greater than min conf
                if confidence > self.MIN_CONF:
                    # Calculating box params
                    center_x = int(detection[0] * IMG_WIDTH)
                    center_y = int(detection[1] * IMG_HEIGHT)

                    w = int(detection[2] * IMG_WIDTH)
                    h = int(detection[3] * IMG_HEIGHT)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    return self.LABELS[class_id],[center_x, center_y, x, y, w, h],float(confidence)

    def identify_objects(self, img):
        outputs = self.model.predict(img)

        objects = []
        confidences = []
        locations = []

        IMG_WIDTH = img.shape[1]
        IMG_HEIGHT = img.shape[0]

        for output in outputs:
            for detection in output:
                # Getting scores
                scores = detection[5:]

                # Finding max score
                class_id = np.argmax(scores)

                # Getting the corresponding confidencce
                confidence = scores[class_id]

                # Create box if the confidence is greater than min conf
                if confidence > self.MIN_CONF:
                    # Calculating box params
                    center_x = int(detection[0] * IMG_WIDTH)
                    center_y = int(detection[1] * IMG_HEIGHT)

                    w = int(detection[2] * IMG_WIDTH)
                    h = int(detection[3] * IMG_HEIGHT)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    objects.append(self.LABELS[class_id])
                    confidences.append(float(confidence))
                    locations.append([center_x, center_y, x, y, w, h])

        return objects,locations,confidences

    def predict_test_image(self):
        self.TEST_IMAGE = imread(self.TEST_IMAGE_FILE_NAME)
        label, box, conf = self.identify_object(self.TEST_IMAGE)
        
        print(f"Identified as : {label}")
        
        plt.imshow(self.draw_box(self.TEST_IMAGE,[box],[label],[conf]))
        plt.show()

    def draw_box(self, img, boxes,objects, confidences):
        for box,object,conf in zip(boxes,objects,confidences):
            x, y, w, h = box[2], box[3], box[4],  box[5]

            color = np.random.choice(self.COLORS,size=1)

            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(object,conf)
            cv2.putText(img, text, (x, y - 5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img

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

def main():
    from mirs_system.ai.vision.calibration.raspberrypi import Raspberrypi

    test_folder = os.path.join(os.path.dirname(__file__),'test')

    obj = ObjectIdentifier()

    rasp = Raspberrypi()
    files,err = rasp.get_sample_image(test_folder)


    if err:
        print(err)
        return
    
    left_img = files[0]
    right_img = files[1]

    print(obj.identify_object(left_img))


if __name__ == "__main__":
    main()