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

def testCamera(index=0):
    cap = cv2.VideoCapture(0)
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        cap.release()
        return False
    cap.release()
    return True

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread(args["image"])
if not image.any():
    print("Couldn't load image")

# TODO: WHY DOESNT THIS WORK!?!?!?
#image = cv2.flip(image,0)

# Image Loading and Resizing
resized = imutils.resize(image, width=1000)
ratio = image.shape[0] / float(resized.shape[0])
hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
print(ratio)

# CONSTANTS
# Offsets to use for all parts Only Using Two Squares RN
greenWorldOrigin = np.array([[200],[525]])
redWorld = np.array([[-200],[525]])
debug = True
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

lowerBlocks =  np.array([10,0,140])
upperBlocks = np.array([47,70,255])

# Jenga block via pixels from our camera
jengaWHigh = 42
jengaWLow = 36
jengaLHigh = 121
jengaLLow = 115

# Create Masks Based on the Bounds
greenMask = cv2.inRange(hsv, lower_green, upper_green)
yellowMask = cv2.inRange(hsv, lower_yellow, upper_yellow)
redMask = cv2.inRange(hsv, lower_red, upper_red)
blockMask = cv2.inRange(hsv, lowerBlocks, upperBlocks)

if debug:
    cv2.imshow("HSV Green", greenMask)  # Tag 1
    cv2.imshow("HSV Yellow", yellowMask) # Yellow Tag 2
    cv2.imshow("HSV Red", redMask) # Red Tag 2

# Wrap some basic OpenCV for Easier Use
def drawContours(image, box, color=(0, 0, 255), thickness=2):
    cv2.drawContours(image,[box],0,color,thickness)

def drawPoint(image, point, color = (0, 255, 255), r = 1):
    cv2.circle(image, (int(point[0]), int(point[1])), r, color, 2)

def drawBasis(image, originPixel, basis, arrowSize = 20, colorX = (0, 255, 0), colorY = (0, 0, 255)):
    originTuple = (int(originPixel[0]), int(originPixel[1]))
    
    basisLarger = arrowSize*basis + originPixel
    endXPosVec = (int(basisLarger[0][0]), int(basisLarger[1][0]))
    endYPosVec = (int(basisLarger[0][1]), int(basisLarger[1][1]))
    print(originTuple)
    print(endXPosVec)
    print(endYPosVec)
    cv2.arrowedLine(image, originTuple, endXPosVec, colorX, 2)
    cv2.arrowedLine(image, originTuple, endYPosVec, colorY, 2)

def getCnts(image):
    cnts = cv2.findContours(image, cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
    return imutils.grab_contours(cnts)

# loop over the contours
def getPixelCenterSquare(cnts):
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
        if debug:
            print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}")

        box = cv2.boxPoints(rot_rect) #* ratio
        box = np.int0(box)

        return (rot_rect[0], box) # Returns the center in pixels and also the box

def calcBasis(baseTagWorld, secondaryTagWorld, baseTagPixel, secondaryTagPixel):
    """ np.array 2x1, np.array 2x1, np.array 2x1, np.array 2x1 """
    tagToTagVectorWorld = secondaryTagWorld-baseTagWorld
    distWorld = np.hypot(tagToTagVectorWorld[0], tagToTagVectorWorld[1]) # Calculates hypotenuse!

    tagToTagVectorPixel = secondaryTagPixel-baseTagPixel
    distPixel = np.hypot(tagToTagVectorPixel[0], tagToTagVectorPixel[1]) 

    frameRotation = math.atan2(tagToTagVectorPixel[1], tagToTagVectorPixel[0])  # This may need to be adjusted since images use weird coordinates

    mmPerPixel = distWorld/distPixel

    xPosVector = -1* tagToTagVectorPixel/distPixel * mmPerPixel
    yPosVector = -1* np.vstack((xPosVector[1], xPosVector[0]))

    basis = np.hstack((xPosVector,yPosVector))
    basisInv = np.linalg.inv(basis)

    return basis, basisInv, frameRotation

def changeBasisAtoB(bOffset, aOffset, fRot, basis, aVector, bRot):
    """
    Expects np.array(mm), np.array(pixels), radians, np.array(2x2)(mm), np.array(pixels), radians
    """
    blockCenterImage = aVector - aOffset
    bVector = np.matmul(basis, blockCenterImage)
    bVector = bVector + bOffset
    rotation = bRot-fRot
    return bVector, rotation 

cntsYellow = getCnts(yellowMask)
cntsRed = getCnts(redMask)
cntsGreen = getCnts(greenMask)

# Calculate the centers of each piece
(greenCenter, greenBox) = getPixelCenterSquare(cntsGreen)
(yellowCenter, yellowBox) = getPixelCenterSquare(cntsYellow)
(redCenter, redBox) = getPixelCenterSquare(cntsRed)

# draw rotated rectangle on copy of img
tags_image = resized.copy()
if greenBox is not None:
    drawContours(tags_image, greenBox)
    drawPoint(tags_image, greenCenter)
if yellowBox is not None:
    drawContours(tags_image, yellowBox)
    drawPoint(tags_image, yellowCenter)
if redBox is not None:
    drawContours(tags_image, redBox)
    drawPoint(tags_image, redCenter)

# Image coords pixels
originTagPixel = np.array([[greenCenter[0]], [greenCenter[1]]])
secondaryTag = np.array([[redCenter[0]], [redCenter[1]]])

basisWorld, basisPixel, frameRotation = calcBasis(greenWorldOrigin, redWorld, originTagPixel, secondaryTag)

drawBasis(tags_image, originTagPixel, basisWorld)

print(f"World: \n{basisWorld}\nPixel: \n{basisPixel}")

####THESE PARAMS NEED TO BE FROM THE BLOCK/OPENCV
blockRotationImage = 0
blockCenterImageFullFrame = np.array([[yellowCenter[0]],
                                      [yellowCenter[1]]])
print("yellow", yellowCenter)
# Example to World Coords
coordWorld, rotationWorld = changeBasisAtoB(greenWorldOrigin, originTagPixel, frameRotation, basisWorld,
                                         blockCenterImageFullFrame, blockRotationImage )
print(f"World Coords: {coordWorld}, World Rotation: {rotationWorld}")

# # Example to Pixel Coords
# testWorldCoords = np.array([[0],[100]])
# coordPixel, rotationBlock = changeBasisAtoB(originTagPixel, greenWorldOrigin , blockRotationImage, basisPixel,
#                                          testWorldCoords, frameRotation  )
# print(f"Pixel Coords:\n {coordPixel}\n Pixel rotation: {rotationBlock}")
# drawPoint(tags_image, coordPixel)


if debug:
    #Display a grid
    for i in range(-500, 500, 50):
        for j in range(0, 650, 50):
            testWorldCoords = np.array([[i],[j]])
            coordPixel, rotationBlock = changeBasisAtoB(originTagPixel, greenWorldOrigin , blockRotationImage, basisPixel,
                                                    testWorldCoords, frameRotation  )
            drawPoint(tags_image, coordPixel)
    cv2.imshow("Image", tags_image)
    cv2.waitKey(0)


# TIME FOR JENGA!!!!!
jengaDebug = True
jengaImg = resized.copy()

if jengaDebug:
    cv2.imshow("HSV Block Mask", blockMask)  # Tag 1
    cv2.waitKey(0)

singleBlock = None
groupBlock = None   #TODO: Implement search for a group and do cool stuff!

def getBlockPixel(mask):
    cnts = getCnts(mask)
    for c in reversed(cnts): # Start from top of frame since I can't seem to flip the image
        rot_rect = cv2.minAreaRect(c)
        rotation = rot_rect[2]
        # small pixel cutoff
        pixelCutoff = 10
        if rot_rect[1][1] < pixelCutoff or rot_rect[1][0] < pixelCutoff: # THIS HANDLES-> or rot_rect[1][1] == 0 or rot_rect[1][0] == 0:
            continue
        blockLong = max(rot_rect[1])*ratio
        blockShort = min(rot_rect[1])*ratio
        if blockLong > jengaLHigh or blockLong < jengaLLow or blockShort > jengaWHigh or blockShort < jengaWLow:
            # This isn't a jenga Block
            continue
            # TODO: LOOK FOR CLUSTERS HERE!!!
        
        # This means the block is in a certain orientation that needs an offset
        if rot_rect[1][0] <= rot_rect[1][1]:
            rotation += 90
        rotation *= -1      # All rotation needs to be adjusted!

        #DEBUG 
        if jengaDebug:
            print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}, rotation: {rotation}")
            print(f"ratio:{ratio}, w: {ratio*rot_rect[1][0]}, h: {ratio*rot_rect[1][1]}")
            print(f"Final rotation: {rotation}")
        
            box = cv2.boxPoints(rot_rect) #* ratio
            box = np.int0(box)
            drawContours(jengaImg, box)
            drawPoint(jengaImg, rot_rect[0], (255,0,0))
            
        return np.array([[rot_rect[0][0]],[rot_rect[0][1]]] ), rotation
    return None

def getBlockWorld(originWorld, originWorldPixel, frameRot, basisW, mask):
    singleBlockCenterPixel, singleBlockRotation =  getBlockPixel(mask)
    #print(f"Block: {singleBlockCenterPixel}, theta: {singleBlockRotation}")
    # Lets take the block into world Coordinates!!!
    coordWorld, rotationWorld = changeBasisAtoB(originWorld, originWorldPixel, frameRot, basisW,
                                            singleBlockCenterPixel, singleBlockRotation)
    #print(f"World Coords: {coordWorld}, World Rotation: {rotationWorld}")
    # TODO: what does this return if nothing has happened??
    return coordWorld, rotationWorld


getBlockWorld(greenWorldOrigin, originTagPixel, frameRotation, basisWorld, blockMask)

cv2.imshow("HSV Block", jengaImg)  # Tag 1
cv2.waitKey(0)