from pandas import read_csv
import numpy as np
from sklearn.utils import shuffle
from matplotlib.image import imread
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.layers import Conv2D, Lambda, Cropping2D, MaxPooling2D

measurments = read_csv('data/driving_log.csv', usecols=[3]).values
C = measurments
L = measurments + 0.2
R = measurments - 0.2

images_C = read_csv('data/driving_log.csv', usecols=[0]).values
images_L = read_csv('data/driving_log.csv', usecols=[1]).values
images_R = read_csv('data/driving_log.csv', usecols=[2]).values 

def genImages(idx, size):
    images = []
    
    for path in images_C[idx:idx+size]:
        fname = 'data/IMG/'+ path[0].split('/')[-1]
        images.append(imread(fname))
        
    for path in images_L[idx:idx+size]:
        fname = 'data/IMG/'+ path[0].split('/')[-1]
        images.append(imread(fname))
        
    for path in images_R[idx:idx+size]:
        fname = 'data/IMG/'+ path[0].split('/')[-1]
        images.append(imread(fname))

    images = np.array(images)
    images = np.concatenate((images, np.fliplr(images)), axis=0)
    
    measurments = np.concatenate((C[idx:idx+size], L[idx:idx+size], R[idx:idx+size],
                                  -C[idx:idx+size], -L[idx:idx+size], -R[idx:idx+size]), axis=0)
    
    X_train, Y_train = shuffle(images, measurments)
    return X_train, Y_train
def model():
    model = Sequential()

    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(160, 320, 3)))
    model.add(Cropping2D(cropping=((70, 25), (0, 0))))

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

    model.compile('adam', "mse")
    
    return model


if __name__ == "__main__":
    model = model()

    size = 2500
    for i in range(10):
        print('='*51)
        print('='*24, i+1, '='*24)
        print('='*51)
        for idx in range(10):
                X_train, Y_train = genImages(idx*size, size)
                model.fit(X_train, Y_train, batch_size=600, epochs=1, shuffle=True, validation_split=0.2)
