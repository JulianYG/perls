## To calculate /base to /RGB transformation from RGB extrinsics:

Given extrinsic RGB translation T, RGB rotation matrix M, 
```
t = - M' x T / 1000.
H = [M |  1]  (convert M to 4 x 4 homogeneous matrix) 
q = tf.transformations.quaternion_from_matrix(H') (x, y, z, w)
```

static transform args := $x $q

t for ada:
```
0.68717610909, 0.10101958479, 0.91475941674
```
q for ada:
```
0.66823929, 0.705541, -0.18376103, -0.14798658
```

t for charles:
```
1.17080859, 0.12849696, 0.63101727
```
q for charles:
```
0.67379288,  0.66603288, -0.22929853, -0.22321635
```

## To calculate /base to /IR transformation from RGB extrinsics and IR_RGB extrinsics:

Given RGB Extrinsics T, R, and IAI calibrated stereo extrinsics,

 


Accurate calibration data for Ada:

* [RGB_Rotation](https://www.dropbox.com/s/0d2xurmqbgyqytz/KinectTracker_rotation.p?dl=0)

* [RGB_Translation](https://www.dropbox.com/s/7md2esnan44h2cg/KinectTracker_translation.p?dl=0)

* [RGB_Intrinsics](https://www.dropbox.com/s/iga6aa42tuioohc/intrinsics.p?dl=0)

* [RGB_Distortion](https://www.dropbox.com/s/au0n6stpv6gubyl/distortion.p?dl=0)

* [IR_Intrinsics]()

* [IR_Distortion]()

* [IR_RGB_Essential]()

* [IR_RGB_Fundamental]()

Accurate calibration data for Charles:

* [RGB_Rotation](https://www.dropbox.com/s/zjp3gk1pgy2hwdg/KinectCalibrator_rotation.p?dl=0)

* [RGB_Translation](https://www.dropbox.com/s/3ooayrmrad47uov/KinectCalibrator_translation.p?dl=0)

* [RGB_Intrinsics](https://www.dropbox.com/s/0f6wcv08mt3g5qu/intrinsics.p?dl=0)

* [RGB_Distortion](https://www.dropbox.com/s/zuvbglapny9n110/distortion.p?dl=0)

* [IR_Intrinsics]()

* [IR_Distortion]()

* [IR_RGB_Essential]()

* [IR_RGB_Fundamental]()




