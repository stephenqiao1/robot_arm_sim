import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import tensorflow as tf
from tensorflow import keras
from keras import layers

import matplotlib.pyplot as plt

cifar10 = keras.datasets.cifar10

(train_images, train_labels), (test_images, test_labels) = cifar10.load_data()

print(train_images.shape)

# Normalize: 0,255 -> 0,1
train_images, test_images = train_images / 255.0, test_images / 255.0

class_names = ['airplane', 'automobile', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck']

# def show():
#     plt.figure(figsize=(10,10))
#     for i in range(16):
#         plt.subplot(4,4,i+1)
#         plt.xticks([])
#         plt.yticks([])
#         plt.grid(False)
#         plt.imshow(train_images[i], cmap=plt.cm.binary)
#         # The CIFAR labels happens to be arrays,
#         # which is why you need the extra index
#         plt.xlabel(class_names[train_labels[i][0]])
#     plt.show()
    
# show()

# model
model = keras.models.Sequential() # initializes a sequential model, indicating that the layers of the model will be arranged in sequence
# 32 filters each of size 3x3, stride of 1x1, the input shape is set to 32x32 pixels with 3 channels
model.add(layers.Conv2D(32, (3, 3), strides=(1, 1), padding="valid", activation='relu', input_shape=(32, 32, 3)))
model.add(layers.MaxPool2D((2,2))) # pooling size of 2x2, reducing the spatial dimensions of the input by half
model.add(layers.Conv2D(32, 3, activation='relu'))
model.add(layers.MaxPool2D((2,2)))
model.add(layers.Flatten())
model.add(layers.Dense(64, activation='relu'))
model.add(layers.Dense(10))
print(model.summary())
# import sys; sys.exit()

# loss and optimizer
loss = keras.losses.SparseCategoricalCrossentropy(from_logits=True)
optim = keras.optimizers.Adam(learning_rate=0.001)
metrics = ["accuracy"]

model.compile(optimizer=optim, loss=loss, metrics=metrics)

# training
batch_size = 64
epochs = 6  

model.fit(train_images, train_labels, epochs=epochs, batch_size=batch_size, verbose=2)

# evaluate
model.evaluate(test_images, test_labels, batch_size=batch_size, verbose=2)