import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import tensorflow as tf
from tensorflow import keras
import numpy as np
import matplotlib.pyplot as plt

mnist = keras.datasets.mnist # loads the MNIST dataset

(x_train, y_train), (x_test, y_test) = mnist.load_data() # splits into training and test sets
print(x_train.shape, y_train.shape)

# normalize: 0,255 -> 0,1
x_train, x_test = x_train / 255.0, x_test / 255.0 # image data preprocessing

# for i in range(6):
#     plt.subplot(2,3,i+1)
#     plt.imshow(x_train[i], cmap='gray')
# plt.show()

# model
model = keras.models.Sequential([
    keras.layers.Flatten(input_shape=(28,28)), # flattens the 28x28 pixel images into 1D arrays
    keras.layers.Dense(128, activation='relu'),
    keras.layers.Dense(10), # 10 units, because the output only has 10 classifications
])

print(model.summary())
# model = keras.Sequential()
# model.add(keras.layers.Flatten(input_shape=(28,28)))
# model.add(keras.layers.Dense(128, activation='relu'))
# model.add(keras.layers.Dense(10))

# loss and optimizer
loss = keras.losses.SparseCategoricalCrossentropy(from_logits=True)
optim = keras.optimizers.Adam(lr=0.001)
metrics = ["accuracy"]

model.compile(loss=loss, optimizer=optim, metrics=metrics)

# training
batch_size = 64
epochs = 5

model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, shuffle=True, verbose=2)

# evaluate
model.evaluate(x_test, y_test, batch_size=batch_size, verbose=2)

# predictions
probability_model = keras.models.Sequential([ # outputs probabilities
    model,
    keras.layers.Softmax()
]) 

predictions = probability_model(x_test) # makes predictions on the test set
pred0 = predictions[0] # extracts the predictions for the first test image
print(pred0)
label0 = np.argmax(pred0) # finds the class with the highest probability
print(label0)

# model + softmax (alternative approach)
predictions = model.predict(x_test, batch_size=batch_size)
predictions = tf.nn.softmax(predictions)
pred0 = predictions[0]
print(pred0)
label0 = np.argmax(pred0)
print(label0)

pred05s = predictions[0:5]
print(pred05s.shape)
label05s = np.argmax(pred05s, axis=1)
print(label05s)