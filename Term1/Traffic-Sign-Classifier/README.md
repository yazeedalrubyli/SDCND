

# Traffic Sign Recognition Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Our main goal in this project is to classify [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset) by using Convolutional Neural Network (CNN). This project is a part of Udacity's Self-Driving Car Nanodegree Projects in Term 1.

## Dataset

```
    Type         Amount     Size     Color
    
    Training     34,799    32x32      RGB

    Validation   12,630    32x32      RGB

    Testing      4,410     32x32      RGB
```

## Preprocessing

Here are some preprocessing techniques that have proven to work on this dataset:

* **Pixcel Normalization**. We normlized the image to a range `[0,1]` instade of `[0-255]`.

* **Pixcel Mean Centering**. We subtract 0.5 from each pixcel after applying the pixcel normalization.

## Augmentation

When we augment all the images at once, the accuracy did not improve that much. I think we should augment only the classes that we do not have much data about it. These are augmentation techniques used: 

* **Flipping**

* **Rotating**

* **Shifting**

* **Zooming**

> **Note**: Augmentation is there in code as a function. It needs some modification as we mentioned above.

## Model 

### Architecture

I started first with LeNet and I tried to play around with the hyperparameters and I got around 89% as validation accuracy. Then, after reading some papers about available CNN I tried to implement [SqueezeNet](https://arxiv.org/pdf/1602.07360.pdf) which they claim to be as accurate as AlexNet but 50X faster. I got validation accuracy about 90%, so I decided to build my own CNN. I build my CNN with 3 Convolutional layers and 3 Fully Connected layers, I got validation accuracy about **99% without even preprocess or augmenting my dataset**, I called my network, **YazNet** ;)

<p align="center">
  <img src="YazNet_Arch.png" alt="Model architecture"/>
</p>

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
    Activation: Sigmoid
```

### Regularization

* **Dropout**. Prevent the model from overfitting.

* **Reduce Learning Rate**. Reduce learning rate when a metric has stopped improving.

* **Early Stopping**. Stop model from continues with the learning process if there is no improvement in the validation accuracy.

## Training

How would I choose the optimizer? What is its Pros & Cons and how would I evaluate it?

How would I decide the number and type of layers?

How would I tune the hyperparameter? How many values should I test and how to decide the values?

How would I preprocess my data? Why do I need to apply a certain technique?

How would I train the model?

How would I evaluate the model? What is the metric? How do I set the benchmark?


## Result

Discuss before doing the actual prediction, what qualities your new images have (e.g: brightness, contrast, etc. ) that might cause your model to misclassify them.

Clearly discuss how certain or uncertain your model is of its prediction.

Compare the accuracy on the new set of images to that on the old test set.

Your explanation can look something like: the accuracy on the captured images is X% while it was Y% on the testing set thus It seems the model is overfitting


## Future Improvements

* **Augmentation for Classes with Less Data**