import os
import sys
import cv2
import numpy as np
from io import BytesIO
from PIL import Image
import torch
from .models.experimental import attempt_load
from .models.common import non_max_suppression

class Model():
    def __init__(self):
        self.loaded = False
        self.WEIGHTS = os.path.join(os.path.dirname(__file__),"yolov-custom.pt")
        self.model = None
        self.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        self.IMAGE_SIZE = 640

    def load(self):
        print(f"Using {self.WEIGHTS} ...")
        self.model = attempt_load(self.WEIGHTS, map_location=self.DEVICE)
        self.loaded = True

    def predict(self, X, feature_names = None, meta = None):
        if not self.loaded:
            self.load()

        if isinstance(X, bytes):
            image = Image.open(BytesIO(X))
            image = np.array(image).astype(np.float32)
        else:
            image = np.array(X[0]).astype(np.float32)
        # Resize image to the inference size
        ori_h, ori_w = image.shape[:2]
        image = cv2.resize(image, (self.IMAGE_SIZE, self.IMAGE_SIZE))
        
        # Transform image from numpy to torch format
        image_pt = torch.from_numpy(image).permute(2, 0, 1).to(self.DEVICE)
        image_pt = image_pt.float() / 255.0

        # Infer
        with torch.no_grad():
            pred = self.model(image_pt[None], augment=False)[0]

        # NMS
        pred = non_max_suppression(pred)[0].cpu().numpy()

        # Resize boxes to the original image size
        pred[:, [0, 2]] *= ori_w / self.IMAGE_SIZE
        pred[:, [1, 3]] *= ori_h / self.IMAGE_SIZE

        return pred
