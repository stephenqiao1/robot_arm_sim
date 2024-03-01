import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import tensorflow as tf
from tensorflow import keras
from keras import layers

mnist = keras.datasets.mnist

(x_train, y_train), (x_test, y_test) = mnist.load_data()
x_train, x_test = x_train / 255.0, x_test / 255.0
# 28, 28
# input_size=28
# seq_length=28
# model
model = keras.models.Sequential()
model.add(keras.Input(shape=(28,28)))
model.add(layers.SimpleRNN(128, activation='relu'))
model.add(layers.Dense(10))