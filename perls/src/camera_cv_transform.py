import cv2
import numpy as np

imgPoints = np.array([[171, 218],
					  [274, 223],
					  [351, 219],
					  [415, 221],
					  [171, 319],
					  [272, 318],
					  [353, 315],
					  [445, 318],
					  [173, 404],
					  [267, 413],
					  [355, 411],
					  [445, 414]], np.float32)
objectPoints = np.array(
    [[0.4957, -0.1641, 0.0945], 
    [0.4947, -0.0416, 0.0920], 
    [0.4921,0.0445,0.0921],
    [0.4835,0.1466,0.0895],
    [0.6132,-0.1623,0.0884],
    [0.6070,-0.0430,0.0922],
    [0.6042,0.0418,0.0916],
    [0.5983,0.1526,0.0873],
    [0.6758,-0.1606,0.0890],
    [0.717,-0.0451,0.0878],
    [0.7135,0.0443,0.0966],
    [0.7175, 0.1570, 0.0880]], np.float32)

distortion = np.array([0.147084, -0.257330, 0.003032, -0.006975, 0.000000], np.float32)

cameraMatrix = np.array([[600.153387, 0, 315.459915], [0, 598.015225, 222.933946], [0, 0, 1]], np.float32)

retval, rvec, tvec = cv2.solvePnP(objectPoints, imgPoints, cameraMatrix, distortion)

rotMatrix = cv2.Rodrigues(rvec)[0]

invRot = np.linalg.inv(rotMatrix)
invCamera = np.linalg.inv(cameraMatrix)

uvPoint = np.array([313,272,1]) # Change the pixel point here (in form [u,v,1])

tempMat = invRot * invCamera
tempMat2 = tempMat.dot(uvPoint)
tempMat3 = invRot.dot(np.reshape(tvec, 3))

s = .09 + tempMat3[2] # .09 + (inv Rotation matrix * inv Camera matrix * point) -> .09 is approx height of each block
s /= tempMat2[2] # inv Rotation matrix * tvec


#s * [u,v,1] = M(R*[X,Y,Z] - t)  ->   R^-1 * (M^-1 * s * [u,v,1] - t) = [X,Y,Z] 
temp = invCamera.dot(s*uvPoint)
temp2 = temp - np.reshape(tvec, 3)

realworldCoords = invRot.dot(temp2)
print realworldCoords