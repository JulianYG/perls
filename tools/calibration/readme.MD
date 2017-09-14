## To calculate /base to /RGB transformation from RGB extrinsics:

Given extrinsic RGB translation `T`, RGB rotation matrix `M`, 
```
t = - M' x T / 1000.
H = [M |  1]  (convert M to 4 x 4 homogeneous matrix) 
q = tf.transformations.quaternion_from_matrix(H') (x, y, z, w)
```

static transform args := `$x $q`

`t` for ada:
```
0.68717610909, 0.10101958479, 0.91475941674
```
`q` for ada:
```
0.66823929, 0.705541, -0.18376103, -0.14798658
```


RGB `t` for charles:
```
1.17080859, 0.12849696, 0.63101727
```
RGB `q` for charles:
```
0.67379288,  0.66603288, -0.22929853, -0.22321635
```
IR `t` for charles:
```
1.17819428  0.14306091  0.61401135
```
IR `q` for charles:
```
0.67648737  0.66054851 -0.24221597 -0.21765094
```

## To calculate /base to /IR transformation from RGB extrinsics and IR_RGB extrinsics:

Given RGB Extrinsics `T`, `R`, and IAI calibrated stereo extrinsics, combine `T`, `R` into a 4x4 homogeneous pose matrix `P`. Also combine the stereo extrinsics into a 4x4 `H`, compute `P x H` and repeat above operations. Follow the example of `compute_transform.py`.


Accurate calibration data for Ada:

* [robot_to_RGB_Rotation](https://www.dropbox.com/s/0d2xurmqbgyqytz/KinectTracker_rotation.p?dl=0)

* [robot_to_RGB_Translation](https://www.dropbox.com/s/7md2esnan44h2cg/KinectTracker_translation.p?dl=0)

* [RGB_Intrinsics](https://www.dropbox.com/s/iga6aa42tuioohc/intrinsics.p?dl=0)

* [RGB_Distortion](https://www.dropbox.com/s/au0n6stpv6gubyl/distortion.p?dl=0)

* [IR_Intrinsics](https://www.dropbox.com/s/xpoodfv35xdk245/IR_intrinsics.p?dl=0)

* [IR_Distortion](https://www.dropbox.com/s/mj0dtvvb06j1c94/IR_distortion.p?dl=0)

* [RGB_to_IR_Rotation](https://www.dropbox.com/s/6qzsyhy8ribgsmq/IR_RGB_rotation.p?dl=0)

* [RGB_to_IR_Translation](https://www.dropbox.com/s/7324szb7w5d5mn6/IR_RGB_translation.p?dl=0)

* [RGB_to_IR_Essential](https://www.dropbox.com/s/jzl8rz1rprqu9gg/IR_RGB_essential.p?dl=0)

* [RGB_to_IR_Fundamental](https://www.dropbox.com/s/zzgir30fooeu232/IR_RGB_fundamental.p?dl=0)



Accurate calibration data for Charles:

* [robot_to_RGB_Rotation](https://www.dropbox.com/s/zjp3gk1pgy2hwdg/KinectCalibrator_rotation.p?dl=0)

* [robot_to_RGB_Translation](https://www.dropbox.com/s/3ooayrmrad47uov/KinectCalibrator_translation.p?dl=0)

* [RGB_Intrinsics](https://www.dropbox.com/s/synzq0t4s41z5v3/RGB_intrinsics.p?dl=0)

* [RGB_Distortion](https://www.dropbox.com/s/8amzijvzg1slvpw/RGB_distortion.p?dl=0)

* [IR_Intrinsics](https://www.dropbox.com/s/ddkpf39xl4450u2/IR_intrinsics.p?dl=0)

* [IR_Distortion](https://www.dropbox.com/s/98wbwukynmyc4mf/IR_distortion.p?dl=0)

* [RGB_to_IR_Rotation](https://www.dropbox.com/s/nw408egw0ukdij8/IR_RGB_rotation.p?dl=0)

* [RGB_to_IR_Translation](https://www.dropbox.com/s/880ugumwckj1pff/IR_RGB_translation.p?dl=0)

* [RGB_to_IR_Essential](https://www.dropbox.com/s/a3dio9mflh66ozl/IR_RGB_essential.p?dl=0)

* [RGB_to_IR_Fundamental](https://www.dropbox.com/s/t5qp7h664rd6c19/IR_RGB_fundamental.p?dl=0)

* [robot_to_IR_rotation](https://www.dropbox.com/s/zocridk90c0ak5r/robot_IR_rotation.p?dl=0)

* [robot_to_IR_translation](https://www.dropbox.com/s/coc2khn481l6jcu/robot_IR_translation.p?dl=0)
