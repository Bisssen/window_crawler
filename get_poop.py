import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import cv2
import glob

# From Keras
from keras.models import Sequential, load_model
from keras.layers import Dense, Activation
from keras.layers.convolutional import Convolution2D
from keras.layers import Conv2D, MaxPooling2D, Flatten, Dropout
from keras.layers.normalization import BatchNormalization
from keras import optimizers

from sklearn.preprocessing import MinMaxScaler


# Loading already trained model (whole model, not only weights)


def get_poop(model, num, image_paths):
    classes = ("clean","dirty")
    # initialize array
    X = np.zeros(shape=(1,150,150,3))
    img = cv2.imread(image_paths[num])
    img_res = cv2.resize(img,(150,150))
    # normalizing picture
    X[0] = img_res/255
    # Predicting class
    pred = model.predict_classes(X)

    return (pred.tolist()[0][0])