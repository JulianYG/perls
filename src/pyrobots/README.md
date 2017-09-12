## To calculate correct transformations:

Given calibrated extrinsic translation T, rotation matrix M, 
```
t = (- M' x T) / 1000.
H = [M |  1]  (convert M to 4 x 4 homogeneous matrix) 
q = tf.transformations.quaternion_from_matrix(H') (w, x, y, z)
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
1.03092698359, 0.09316863157, 0.76020784767
```
q for charles:
```
0.703789173, 0.710379928, 0.00641113624, 0.000230939802
```
