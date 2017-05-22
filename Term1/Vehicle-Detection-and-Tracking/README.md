# Vehicle Detection and Tracking [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Our goal in this project is to write a software pipeline to identify vehicles in a video from a front-facing camera on a car. 

## Dataset
Datasets are comprised of images taken from the [GTI](http://www.gti.ssr.upm.es/data/Vehicle_database.html) vehicle image database.

|    Type      | Amount |  Size | Color |
|--------------|--------|-------|-------|
| Vehicles     | ~8,790 | 64x64 |  RGB  |
| Non-Vehicles | ~8,970 | 64x64 |  RGB  |

## Pipeline

### Histogram of Oriented Gradients (HOG)
[HOG](http://lear.inrialpes.fr/people/triggs/pubs/Dalal-cvpr05.pdf) has been used as an effective approach for object detection. HOG features were extracted using Scikit library. We converted `RGB` color space to `YCrCb` which gives the best accuracy on test images. After experimenting with HOG parameters, these parameters have chosen `orientations= 9, pixels_per_cell= (8, 8), cells_per_block= (2, 2)` as it tend to be very effective on `64x64` images and increasing values like orientations will increase the features vector and did not improve the model accuracy.
```python
from skimage.feature import hog

image = cv2.cvtColor(image, cv2.COLOR_RGB2YCrCb)

ch1 = feature_image[:,:,0]
ch2 = feature_image[:,:,1]
ch3 = feature_image[:,:,2]

hog1 = hog(ch1, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2))
hog2 = hog(ch2, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2))
hog3 = hog(ch3, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2))

hog_features = np.hstack((hog1, hog2, hog3))
```
<p align="center">
  <img src="Media/car-and-hog.jpg" width="500"/>
  <br/>
  <a href="http://www.udacity.com/drive">SDCND</a>
</p>

### The Classifier
SVM used as a classifier to detect vehicles from non-vehicles, and the accuracy was 99.06% on test samples which represent 20% of the dataset. Matplotlib already imported `png` images as scaled from `0-1` so not need to divide images by 255. Features used to classify images was spatial color, color histogram and HOG features.
```python
from sklearn.preprocessing import StandardScaler
import glob

# Read in our vehicles and non-vehicles
non_vehicles = glob.glob('Dataset/non-vehicles/*/*.png')
vehicles = glob.glob('Dataset/vehicles/*/*.png')

X = []

for path in np.append(vehicles, non_vehicles):
    image = mpimg.imread(path)
    X.append(extract_features(image))
X = np.array(X).astype(np.float64)
Y = np.hstack((np.ones(len(vehicles)), np.zeros(len(non_vehicles))))
# Fit a per-column scaler
X_scaler = StandardScaler().fit(X)
# Apply the scaler to X
scaled_X = X_scaler.transform(X)

rand_state = np.random.randint(0, 100)
X_train, X_test, y_train, y_test = train_test_split(
    scaled_X, Y, test_size=0.2, random_state=rand_state)

svc = LinearSVC()
svc.fit(X_train, y_train)
print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))
```

### Sliding Window Search

```python

```
<p align="center">
  <img src="Media/pre_warp.png"/>
</p>

### False Positives and Overlapping

```python

```
<p align="center">
  <img src="Media/pre_warp.png"/>
</p>

## Result

<p align="center">
  <img src="Media/result.gif" alt="Vehicle Detection and Tracking"/>
  <br/><br/>
  Vehicle Detection and Tracking (<a target="_blank" href="https://youtu.be/TAdXKc_fqCE">Full Video</a>)
</p>

## Discussion


## Future Improvments
* Using [Single Shot MultiBox Detector](https://arxiv.org/abs/1512.02325) (state-of-the-art for object detection)
