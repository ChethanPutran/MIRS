from mediapipe import solutions
import cv2
import numpy as np
import os
from matplotlib.image import imread
import matplotlib.pyplot as plt
TEST_CLASSES = ['pen', 'pencil', 'cup']


class ObjectIdentifier:
    def __init__(self, MIN_CONF=0.5):
        self.MIN_CONF = MIN_CONF
        self.LABELS = []
        self.DIR_NAME = os.path.dirname(os.path.realpath(__file__))
        self.CONFIG_FILE = os.path.join(self.DIR_NAME, "model\\yolov3.cfg")
        self.WEIGHT_FILE = os.path.join(self.DIR_NAME, "model\\yolov3.weights")
        self.read_labels()
        self.net = cv2.dnn.readNetFromDarknet(
            self.CONFIG_FILE, self.WEIGHT_FILE)
        # self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)

    def read_labels(self):
        with open(os.path.join(self.DIR_NAME, "model\\labels.txt"), 'r') as file:
            lines = file.readlines()
            for line in lines:
                self.LABELS.append(line.strip())

    def identify_common_objects(self, img):

        outputs = self.predict(img)

        IMG_WIDTH = img.shape[1]
        IMG_HEIGHT = img.shape[0]

        class_ids = []
        confidences = []
        locations = []

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

                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    locations.append([center_x, center_y, x, y, w, h])

        return class_ids, locations, confidences

    def identify_object(self, img,labels):
        output = self.predict(img)[0]

        IMG_WIDTH = img.shape[1]
        IMG_HEIGHT = img.shape[0]

        for detection in output:
            # Getting scores
            scores = detection[5:]

            # Finding max score
            class_id = np.argmax(scores)

            if self.LABELS[class_id] not in labels :
                continue

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

                return self.LABELS[class_id], [center_x, center_y, x, y, w, h], float(confidence)
        return None, None, None

    def predict_test_image(self):
        self.TEST_IMAGE_FILE_NAME = os.path.join(
            self.DIR_NAME, "test_object.jpg")
        self.TEST_IMAGE = imread(self.TEST_IMAGE_FILE_NAME)
        plt.imshow(self.TEST_IMAGE)
        plt.show()
        label, box, conf = self.identify_object(self.TEST_IMAGE)
        print(f"Identified as : {label}")

    def predict(self, img):
        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]

        # Construct a blob from the image
        blob = cv2.dnn.blobFromImage(
            img, 1/255.0, (416, 416), swapRB=True, crop=False)

        # Setting input
        self.net.setInput(blob)

        # Predicting outputs
        return self.net.forward(ln)

    def identify(self, img, outputs):
        locations = []
        confidences = []
        class_ids = []

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

                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    locations.append([center_x, center_y, x, y, w, h])

        return class_ids, locations, confidences

    def draw_box(self, img, boxes, confidences, class_ids):
        colors = np.random.randint(
            0, 255, size=(len(self.LABELS), 3), dtype='uint8')
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.MIN_CONF, self.MIN_CONF-0.1)

        if len(indices) > 0:
            for i in indices.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = [int(c) for c in colors[class_ids[i]]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(
                    self.LABELS[class_ids[i]], confidences[i])
                cv2.putText(img, text, (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return img

    # def objectron(self):
    #     mp_objectron = solutions.
    #     mp_drawing_utils = solutions.drawing_utils 

    #     objectron = mp_objectron.Objectron(static_image_mode=True,
    #                                        max_num_objects=5, min_detection_confidence=0.2, model_name='Fingure'
    #                                        )
