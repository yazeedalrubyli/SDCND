

# Build a Traffic Sign Recognition Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Our main goal in this project is to classify [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset) by using Convolutional Neural Network (a.k.a CNN). This project is a part of Udacity's Self-Driving Car Nanodegree Projects in Term 1.

## Dataset

The Dataset consists of training, validation and testing datasets. Training dataset consists of 34,799 32x32 color images, validation dataset consists of 4,410 32x32 color images and testing dataset consists of 12,630 32x32 color images.

## Preprocessing

Grayscale, then apply **Contrast Limited Adaptive Histogram Equalization** (a.k.a CLAHE) on dataset seems to provide better accuracy. Finaly, we scale pixle value from `[0-255]` to `[0-1]`.

## Augmentation

When we augment all the images at once, the accuracy did not improve that much. I think the problem is that we should augment only the classes that we do not have much data about it.

### Flipping

### Rotation

### Shifting

### Zooming


## Model 

### Architecture

I started first with LeNet and I tried to play around with the hyperparamters and I got around 89% as validation accurecy. Then, after reading some papers about avalible CNN I tried to impolement [SqueezeNet](https://arxiv.org/pdf/1602.07360.pdf) which they claim to be as accuret as AlexNet but 50X faster. I got validation accurecy about 90%, so I decided to build my own CNN. I build my CNN with 3 Convloutional layers and 3 Fully Connected layers, I got validation accurecy about **98% without even agument or preprocess my datasets**, I called my network, YazNet ;)

```
    Convolution (128, 5x5)
    Activation: Relu
    Max Pooling (2x2)
    Batch Normalization
    
    Convolution (256, 5x5)
    Activation: Rrelu
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

* **Dropout**. Prevent the model from overfiting.

* **Early Stopping**. Stop model from countinue learning process if there is not improvments to the validation accurecy.

## Future Improvments

### Classifier Improvments

* **Agument Classes with Lower Data**.


### Dataset

* **Add New Data**. For testing how good is my model, as 5 new images is not enough to decide.


### Documentation Improvments

* **Define some terms in details**.

* **Add Model Architecture as Figuer**.

* **Training Discussion**.

* **Result Discussion**.

