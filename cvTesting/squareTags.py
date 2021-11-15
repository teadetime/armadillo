# import the necessary packages
import argparse
import imutils
import numpy as np
import cv2
from numpy.core.shape_base import hstack
import math
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
    help="path to the input image")
args = vars(ap.parse_args())

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread(args["image"])
if not image.any():
    print("Couldn't load image")

# Image Loading and Resizing
resized = imutils.resize(image, width=1000)
ratio = image.shape[0] / float(resized.shape[0])
hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
print(ratio)

# CONSTANTS
debug = False
widthHeightLow = 0.9
widthHeightHigh = 1.1
expectedSize = 31   # TODO Change This in Pixels

# Boundaries for the different  Colors
lower_red = np.array([0,150,180])
upper_red = np.array([22,240,240])

lower_green = np.array([66,82,77])
upper_green = np.array([87,208,190])

lower_yellow = np.array([26,43,240])
upper_yellow = np.array([43,110,255])

lowerBlocks =  np.array([16,0,216])
upperBlocks = np.array([43,81,255])

# Create Masks Based on the Bounds
greenMask = cv2.inRange(hsv, lower_green, upper_green)
yellowMask = cv2.inRange(hsv, lower_yellow, upper_yellow)
redMask = cv2.inRange(hsv, lower_red, upper_red)

cv2.imshow("HSV Green", greenMask)  # Tag 1
cv2.imshow("HSV Yellow", yellowMask) # Yellow Tag 2
cv2.imshow("HSV Red", redMask) # Red Tag 2


def getCnts(image):
    cnts = cv2.findContours(image, cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    return imutils.grab_contours(cnts)

# loop over the contours
def getPixelCenter(cnts):
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        rot_rect = cv2.minAreaRect(c)

        # This isn't Valid
        if rot_rect[1][0] == 0 or rot_rect[1][1] == 0:
            continue
        
        # Get Rid of too Big
        #if rot_rect[1][0] > 1.2 * expectedSize or rot_rect[1][0] < .8 * expectedSize or  rot_rect[1][1] > 1.2 * expectedSize or rot_rect[1][1] < .8 * expectedSize:
        #    continue

        widthHeightRatio = rot_rect[1][0]/rot_rect[1][1] 
        if widthHeightRatio > widthHeightHigh or widthHeightRatio < widthHeightLow:
            #Add a check to see if it about the right size!!
            continue

        #rotation = rot_rect[2]
        print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}")

        box = cv2.boxPoints(rot_rect) #* ratio
        box = np.int0(box)

        return (rot_rect[0], box) # Returns the center in pixels and also the box

cntsYellow = getCnts(yellowMask)
cntsRed = getCnts(redMask)
cntsGreen = getCnts(greenMask)

# Calculate the centers of each piece
(greenCenter, greenBox) = getPixelCenter(cntsGreen)
(yellowCenter, yellowBox) = getPixelCenter(cntsYellow)
(redCenter, redBox) = getPixelCenter(cntsRed)

# draw rotated rectangle on copy of img
tags_image = resized.copy()
if greenBox is not None:
    cv2.drawContours(tags_image,[greenBox],0,(0,0,255),2)
if yellowBox is not None:
    cv2.drawContours(tags_image,[yellowBox],0,(0,0,255),2)
if redBox is not None:
    cv2.drawContours(tags_image,[redBox],0,(0,0,255),2)




# Offsets to use for all parts Only Using Two Squares RN
greenWorld = np.array([[-200],[525]])
redWorld = np.array([[200],[525]])
offset = greenWorld-redWorld
distWorld = np.hypot(offset[0], offset[1] ) # Calculates hypotenuse!
print(distWorld)


# Recognize tag1
# Image coords pixels
tag1Center = np.array([[greenCenter[0]], 
                      [greenCenter[1]]])


# Image coords pixels
u2 = 120
v2 = 40
tag2Center = np.array([[redCenter[0]], 
                      [redCenter[1]]])

tagOffset = tag2Center - tag1Center
distPixel = np.hypot(tagOffset[0], tagOffset[1] ) # Calculates hypotenuse!

frameRotation = math.atan2(tagOffset[1], tagOffset[0])  # This may need to be adjusted since images use weird coordinates

mmPerPixel = distWorld/distPixel
vectorRight = tagOffset/distPixel * mmPerPixel
vectorDown = np.vstack((vectorRight[1], vectorRight[0]))

print(f"test Normal should be {mmPerPixel}")
testNormal = np.hypot(vectorRight[0], vectorRight[1] ) # Calculates hypotenuse!
print(testNormal)

basisWorld = np.hstack((vectorRight,vectorDown))


####THESE PARAMS NEED TO BE FROM THE BLOCK/OPENCV
blockRotationImage = 0
blockCenterImageFullFrame = np.array([[yellowCenter[0]],
                                      [yellowCenter[1]]])
print("yellow", yellowCenter)
blockCenterImage = blockCenterImageFullFrame - tag1Center

coordinatesWorld = np.matmul(basisWorld, blockCenterImage)
coordinatesWorld = coordinatesWorld + greenWorld
rotationWorld = blockRotationImage-frameRotation


print(f"Coords: {coordinatesWorld}, Rotation: {rotationWorld}")



if debug:
    cv2.imshow("Image", tags_image)
    cv2.waitKey(0)