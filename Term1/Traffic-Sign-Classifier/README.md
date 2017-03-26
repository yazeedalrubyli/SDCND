

# Build a Traffic Sign Recognition Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Our main goal in this project is to classify [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset) by using Convolutional Neural Network (a.k.a CNN). This project is a part of Udacity's Self-Driving Car Nanodegree Projects in Term 1.

## Dataset

The Dataset consists of training, validation and testing datasets. Training dataset consists of 34,799 32x32 color images, validation dataset consists of 4,410 32x32 color images and testing dataset consists of 12,630 32x32 color images.

## Preprocessing

Grayscale, then apply **Contrast Limited Adaptive Histogram Equalization** (a.k.a CLAHE) on dataset seems to provide better accuracy. Finally, we scale pixel value from `[0-255]` to `[0-1]`.

## Augmentation

When we augment all the images at once, the accuracy did not improve that much. I think the problem is that we should augment only the classes that we do not have much data about it.

* **Flipping**

* **Rotating**

* **Shifting**

* **Zooming**


## Model 

### Architecture

I started first with LeNet and I tried to play around with the hyperparameters and I got around 89% as validation accuracy. Then, after reading some papers about available CNN I tried to implement [SqueezeNet](https://arxiv.org/pdf/1602.07360.pdf) which they claim to be as accurate as AlexNet but 50X faster. I got validation accuracy about 90%, so I decided to build my own CNN. I build my CNN with 3 Convolutional layers and 3 Fully Connected layers, I got validation accuracy about **99% without even preprocessing or augmenting my dataset**, I called my network, YazNet ;)

```
    Convolution (128, 5x5)
    Activation: Relu
    Max Pooling (2x2)
    Batch Normalization
    
    Convolution (256, 5x5)
    Activation: Relu
    Max Pooling (2x2)
    Batch Normalization
    
    Convolution (512, 3x3)
    Activation: Relu
    Max Pooling (2x2)
    Batch Normalization

    Global Average Pooling
    
    FC 512
    Activation: Relu
    Dropout
    
    FC 256
    Activation: Relu
    Dropout
    
    FC 128
    Activation: Relu
    Dropout
    
    FC 43
    Activation('sigmoid')
```

### Regularization

* **Dropout**. Prevent the model from overfitting

* **Reduce Learning Rate**. Reduce learning rate when a metric has stopped improving.

* **Early Stopping**. Stop model from continues with the learning process if there is no improvement in the validation accuracy.

## Future Improvments

### Classifier Improvments

* **Augmenting Classes with Lower Data**


### Dataset

* **Add New Data**. For testing how good is my model, as 5 new images are not enough to decide.


### Documentation Improvments

* **Define some terms in details**

* **Add Model Architecture as Figuer**

* **Training Discussion**

* **Result Discussion**