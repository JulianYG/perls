## To calculate correct transformations:

Given calibrated extrinsic translation T, rotation matrix M, 
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
