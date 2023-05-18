import os
from ultralytics import YOLO


class Model():
    def __init__(self):
        self.WEIGHTS = os.path.join(os.path.dirname(__file__),"best.pt")
        
        # Load a model
        self.model = YOLO(self.WEIGHTS)
        self.names = self.model.names

    def predict(self,img):
        results = self.model.predict(img,verbose=False)  # predict on an image
        return results


