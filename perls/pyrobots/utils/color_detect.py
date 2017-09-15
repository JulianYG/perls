import numpy as np
import cv2
from cv2 import *

im = cv2.imread('test.jpg')

im = cv2.bilateralFilter(im,9,75,75)
im = cv2.fastNlMeansDenoisingColored(im,None,10,10,7,21)
hsv_img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)   # HSV image

###################################
### YELLOW
###################################
# COLOR_MIN = np.array([20, 100, 100],np.uint8)       # HSV color code lower and upper bounds
# COLOR_MAX = np.array([30, 255, 255],np.uint8)       # color yellow
#
# frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)     # Thresholding image
# ret, thresh = cv2.threshold(frame_threshed,127,255,0)

###################################
### RED
###################################
# COLOR_MIN = np.array([0,50,50],np.uint8)
# COLOR_MAX = np.array([10,255,255],np.uint8)
# mask0 = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
#
# COLOR_MIN = np.array([170,50,50],np.uint8)
# COLOR_MAX = np.array([180,255,255],np.uint8)
# mask1 = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
#
# frame_threshed = mask0 + mask1
# ret, thresh = cv2.threshold(frame_threshed,127,255,0)

###################################
### BLUE
###################################
COLOR_MIN = np.array([110, 50, 50], dtype=np.uint8)
COLOR_MAX = np.array([130, 255, 255], dtype=np.uint8)

frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)     # Thresholding image
ret, thresh = cv2.threshold(frame_threshed,127,255,0)


image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

for k, cnt in enumerate(contours):
    # print(cnt)
    print(k)
    x, y, w, h = cv2.boundingRect(cnt)
    # print x,
    # print y
    cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)

cv2.imshow("Show",im)
cv2.imwrite("extracted.jpg", im)
cv2.waitKey()
cv2.destroyAllWindows()



