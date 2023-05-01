import os,cv2,math,random
import numpy as np
import datetime as dt
import tensorflow as tf
from collections import deque
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split


import tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'

# from tensorflow.keras.layers import *
# from .keras.layers import all
# from tensorflow.keras.models import Sequential
# from tensorflow.keras.utils import to_categorical,plot_model
# from tensorflow.keras.callbacks import EarlyStopping
import pandas as pd

class Model():
    def __init__(self):
        self.WEIGHTS = os.path.join(os.path.dirname(__file__),"model.pt")
        self.INPUT_IMG_HEIGHT = 100
        self.INPUT_IMG_WIDTH = 100
        self.SEQ_LEN = 10
        DATA_DIR = 'data'
        self.CLASSES = ['close','open','pick','pitch','yaw','roll','place']
        self.n_classes = len(self.CLASSES)
        #self.model = self.create_model()
        #self.load_model()

    def predict(self,video):
        return self.model(video)
        
    def load_model(self):
        self.model.load_weights(self.WEIGHTS)

    def create_model(self):
        model = tf.keras.layers.C
        
        # [(W−K+2P)/S] + 1
        # size =2,stride = 2 (shift kernal to 2px)
        # W is the input volume(),K is the Kernel size(5),P is the padding (0),S is the stride (2)
        # input_shape = (10,100,100,3)
        
        #1
        model.add(tf.keras.layers.ConvLSTM2D(filters=4,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True,input_shape=(self.SEQ_LEN,self.INPUT_IMG_HEIGHT,self.INPUT_IMG_WIDTH,3)))
        model.add(tf.keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        model.add(tf.keras.layers.TimeDistributed(tf.keras.layers.Dropout(0.2)))
        
        #2
        model.add(tf.keras.layers.ConvLSTM2D(filters=8,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        model.add(tf.keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        model.add(tf.keras.layers.TimeDistributed(tf.keras.layers.Dropout(0.2)))
        
        #3
        model.add(tf.keras.layers.ConvLSTM2D(filters=14,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        model.add(tf.keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        model.add(tf.keras.layers.TimeDistributed(tf.keras.layers.Dropout(0.2)))
        
        #4
        model.add(tf.keras.layers.ConvLSTM2D(filters=16,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        model.add(tf.keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        #model.add(TimeDistributed(Dropout(0.2)))
        
        #Flattening
        model.add(tf.keras.layers.Flatten())
        
        #Dense layer
        model.add(tf.keras.layers.Dense(self.n_classes,activation="softmax"))

        return model
        