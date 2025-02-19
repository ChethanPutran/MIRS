import os
import numpy as np
import tensorflow as tf

class ActionIdentifier:
    def __init__(self,verbose=True):
        self.IMG_HEIGHT = 224
        self.IMG_WIDTH = 224
        self.SEQ_LEN = 10
        self.CLASSES = ['move','pick','place']
        self.n_classes = len(self.CLASSES)
        self.model_path = os.path.join(os.path.dirname(__file__),"model.h5")

        if verbose:
            print("Loading model weights ...")

        self.model = tf.keras.models.load_model(self.model_path,compile=False)

        if verbose:
            print("Done.")
       
     

    """
        Pad and resize an image from a video. 
        Returns Formatted frame with padding of specified output size.

    """
    def pre_process_frames(self,frames):
      frames = np.empty((self.SEQ_LEN,self.IMG_HEIGHT,self.IMG_WIDTH,  3), np.dtype('float32'))
      
      for i,frame in enumerate(frames):
          frame = tf.image.convert_image_dtype(frame, tf.float32)
          frame = tf.image.resize_with_pad(frame, self.IMG_HEIGHT,self.IMG_WIDTH)
          frames[i] = frame

      return frames.reshape(1,self.SEQ_LEN,self.IMG_HEIGHT,self.IMG_WIDTH,  3)

    def predict(self,frame_group):
        data = self.pre_process_frames(frame_group)
        prediction = self.model.predict(data,verbose=False)
        class_idx = prediction.argmax()
        return self.CLASSES[class_idx]
