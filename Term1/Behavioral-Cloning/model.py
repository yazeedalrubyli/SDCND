from pandas import read_csv
import numpy as np
from sklearn.utils import shuffle
from matplotlib.image import imread
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.layers import Conv2D, Lambda, Cropping2D, MaxPooling2D

measurments = read_csv('data/driving_log.csv', usecols=[3]).values
C = measurments # Center Camera Measurments
L = measurments + 0.2 # Left Camera Measurments
R = measurments - 0.2 # Right Camera Measurments

# Center Camera Images Paths
images_C = read_csv('data/driving_log.csv', usecols=[0]).values
# Left Camera Images Paths
images_L = read_csv('data/driving_log.csv', usecols=[1]).values
# Right Camera Images Paths
images_R = read_csv('data/driving_log.csv', usecols=[2]).values 

# Data Generator Method
def genImages(idx, size):
    images = []
    
    # import center images
    for path in images_C[idx:idx+size]:
        fname = 'data/IMG/'+ path[0].split('/')[-1]
        images.append(imread(fname))
    
    # import left images
    for path in images_L[idx:idx+size]:
        fname = 'data/IMG/'+ path[0].split('/')[-1]
        images.append(imread(fname))
    
    # import right images
    for path in images_R[idx:idx+size]:
        fname = 'data/IMG/'+ path[0].split('/')[-1]
        images.append(imread(fname))
    
    # convert images list to numpy array
    images = np.array(images)
    
    # flip all images and add them together
    images = np.concatenate((images, np.fliplr(images)), axis=0)
    
    # add measurments for:
    # original images (C, L, R)
    # flipped images (-C, -L, -R)
    measurments = np.concatenate((C[idx:idx+size], L[idx:idx+size], R[idx:idx+size],
                                  -C[idx:idx+size], -L[idx:idx+size], -R[idx:idx+size]), axis=0)
    
    # shuffle
    X_train, Y_train = shuffle(images, measurments)
    return X_train, Y_train

def model():
    #Keras Model Inisilization
    model = Sequential()
    
    # Pixel Normalization [-1, 1]
    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(160, 320, 3)))
    
    # Cropping image to get ride of unwanted stuff (e.g trees, revier)
    model.add(Cropping2D(cropping=((70, 25), (0, 0))))
    
    ##### Start - NVIDIA Architecture #####
    model.add(Conv2D(24, (5, 5), strides=(2, 2), activation='relu'))
    model.add(Conv2D(36, (5, 5), strides=(2, 2), activation='relu'))
    model.add(Conv2D(48, (5, 5), strides=(2, 2), activation='relu'))
    model.add(Conv2D(64, (3, 3), activation='relu'))
    model.add(Conv2D(64, (3, 3), activation='relu'))

    model.add(Flatten())

    model.add(Dense(100, activation='relu'))
    model.add(Dropout(0.5))

    model.add(Dense(50, activation='relu'))
    model.add(Dropout(0.5))

    model.add(Dense(10, activation='relu'))
    model.add(Dropout(0.5))

    model.add(Dense(1))
    
    ##### End - NVIDIA Architecture #####

    model.compile('adam', "mse")
    
    return model


if __name__ == "__main__":
    model = model()

    size = 2500
    rang = 10
    for i in range(rang):
        print('='*51)
        print('='*24,"%d/%d" % (i+1,rang), '='*24)
        print('='*51)
        for idx in range(rang):
                X_train, Y_train = genImages(idx*size, size)
                model.fit(X_train, Y_train, batch_size=600, epochs=1, shuffle=True, validation_split=0.2)
