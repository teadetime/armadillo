import numpy as np
import cv2 as cv
im = cv.imread('C:\\dev\\git\\armadillo\\cvTesting\\test_frames\\frame_7.png')
imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

print(contours)
print(type(contours))
print(type(contours[0]))
print(type(contours[0][0]))
print(type(contours[0][0][0]))
print(type(contours[0][0][0][0]))

# cv.drawContours(im, contours, -1, (0,255,0), 3)
# cv.imshow("hello world", im)
# cv.waitKey()

