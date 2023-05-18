import os
import numpy as np
from tensorflow import keras
import tensorflow as tf

class ActionIdentifierModel():
    def __init__(self):
        self.IMG_HEIGHT = 224
        self.IMG_WIDTH = 224
        self.SEQ_LEN = 5
        self.CLASSES = ['hold','move','pick','place']
        self.class_ids_for_name = dict([(idx, name) for idx, name in enumerate(self.CLASSES)])
        self.WEIGHTS = os.path.join(os.path.dirname(__file__),"model2.h5")
        self.n_classes = len(self.CLASSES)
        self.model = self.create_model()
        self.load_model()

     
    def load_model(self):
        self.model.load_weights(self.WEIGHTS)

    def create_model(self):
        self.model =keras.Sequential()

        self.model.add(keras.layers.ConvLSTM2D(filters=4,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True,input_shape=(self.SEQ_LEN,self.IMG_HEIGHT,self.IMG_WIDTH,3)))
        self.model.add(keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        self.model.add(keras.layers.TimeDistributed(keras.layers.Dropout(0.2)))
        
        #2
        self.model.add(keras.layers.ConvLSTM2D(filters=8,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        self.model.add(keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        self.model.add(keras.layers.TimeDistributed(keras.layers.Dropout(0.2)))
        
        #3
        self.model.add(keras.layers.ConvLSTM2D(filters=14,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        self.model.add(keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        self.model.add(keras.layers.TimeDistributed(keras.layers.Dropout(0.2)))
        
        #4
        self.model.add(keras.layers.ConvLSTM2D(filters=16,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        self.model.add(keras.layers.MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        #model.add(TimeDistributed(Dropout(0.2)))
        
        #Flattening
        self.model.add(keras.layers.Flatten())
        
        #Dense layer
        self.model.add(keras.layers.Dense(len(self.CLASSES),activation="softmax"))

        return self.model

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
        print(prediction)
        print(class_idx)
        return self.class_ids_for_name[class_idx]
