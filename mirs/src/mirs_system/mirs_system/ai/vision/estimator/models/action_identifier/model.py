import os
import numpy as np
from tensorflow.keras.layers import *

class ActionIdentifierModel():
    def __init__(self):
        self.IMG_HEIGHT = 224
        self.IMG_WIDTH = 224
        self.SEQ_LEN = 5
        self.CLASSES = ['hold','move','pick','place']
        self.class_ids_for_name = dict((name, idx) for idx, name in enumerate(self.CLASSES))
        self.WEIGHTS = os.path.join(os.path.dirname(__file__),"model.h5")
        self.n_classes = len(self.CLASSES)
        self.model = self.create_model()
        self.load_model()

     
    def load_model(self):
        self.model.load_weights(self.WEIGHTS)

    def create_model(self):
        self.model = Sequential()

        self.model.add(ConvLSTM2D(filters=4,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True,input_shape=(SEQ_LEN,IMG_HEIGHT,IMG_WIDTH,3)))
        self.model.add(MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        self.model.add(TimeDistributed(Dropout(0.2)))
        
        #2
        self.model.add(ConvLSTM2D(filters=8,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        self.model.add(MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        self.model.add(TimeDistributed(Dropout(0.2)))
        
        #3
        self.model.add(ConvLSTM2D(filters=14,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        self.model.add(MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        self.model.add(TimeDistributed(Dropout(0.2)))
        
        #4
        self.model.add(ConvLSTM2D(filters=16,kernel_size=(3,3),activation='tanh',
                             data_format="channels_last",recurrent_dropout=0.2,
                             return_sequences=True))
        self.model.add(MaxPooling3D(pool_size=(1,2,3),padding="same",data_format="channels_last"))
        #model.add(TimeDistributed(Dropout(0.2)))
        
        #Flattening
        self.model.add(Flatten())
        
        #Dense layer
        self.model.add(Dense(len(CLASSES),activation="softmax"))
        
    
    def predict(self,frame_group):
        data = np.array(frame_group).reshape(1,self.SEQ_LEN,self.IMG_HEIGHT,self.IMG_WIDTH,3)
        class_idx = self.model.predict(data).argmax()
        return self.class_ids_for_name[class_idx]
