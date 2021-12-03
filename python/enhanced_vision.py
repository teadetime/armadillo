import argparse
import imutils
import numpy as np
import cv2
import math
from time import sleep


# Jenga block sizes under dilated image

def enhance_block_detection(blockMask):
    # returns a pre-processed image that is optimized (optimization TBD) for block detection

    # create a kernel for performing erosion - the size and shape of the kernel determine the properties of erosion
    kernel = np.ones((4,4), np.uint8)
    eroded = cv2.erode(blockMask, kernel, iterations=3) # more iterations means more eroded area
    return eroded

def enhance_vision(blockMask):

    # convert to a color image for displaying boundaries
    jengaWHighDilated = 100
    jengaWLowDilated = 5
    jengaLHighDilated = 190
    jengaLLowDilated = 5

    # display adjusted image
    adjusted_image = cv2.cvtColor(blockMask, cv2.COLOR_GRAY2BGR) # turn into a color image to show colored bounding boxes
    cv2.drawContours(adjusted_image,[box], 0, (0, 255, 255), 2)
    cv2.imshow("Adjusted Image", adjusted_image)

    blockMask = eroded

def is_block(blockLong, blockShort):
    jengaWHighDilated = 100
    jengaWLowDilated = 5
    jengaLHighDilated = 190
    jengaLLowDilated = 5

    res = blockLong < jengaLHighDilated and   \
            blockLong > jengaLLowDilated and    \
            blockShort < jengaWHighDilated and  \
            blockShort > jengaWLowDilated

    return res

def getBlockPixel(self):
    cnts = self.getCnts(enhance_block_detection(self.blockMask))
    for c in reversed(cnts): # Start from top of frame since I can't seem to flip the image
        rot_rect = cv2.minAreaRect(c)
        rotation = rot_rect[2]
        # small pixel cutoff
        pixelCutoff = 10
        if rot_rect[1][1] < pixelCutoff or rot_rect[1][0] < pixelCutoff: # THIS HANDLES-> or rot_rect[1][1] == 0 or rot_rect[1][0] == 0:
            continue
        blockLong = max(rot_rect[1])*self.ratio
        blockShort = min(rot_rect[1])*self.ratio

        if not is_block(blockLong, blockShort):
            continue

        # This means the block is in a certain orientation that needs an offset
        if rot_rect[1][0] <= rot_rect[1][1]:
            rotation += 90
        rotation *= -1      # All rotation needs to be adjusted!

        #DEBUG
        if self.jengaDebug:
            print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}, rotation: {rotation}")
            print(f"ratio:{self.ratio}, w: {self.ratio*rot_rect[1][0]}, h: {self.ratio*rot_rect[1][1]}")
            print(f"Final rotation: {rotation}")

            box = cv2.boxPoints(rot_rect) #* ratio
            box = np.int0(box)
            self.drawContours(box)
            self.drawPoint(rot_rect[0], (255,0,0))

        return np.array([[rot_rect[0][0]],[rot_rect[0][1]]] ), rotation
    return None





    print("this isn't a jenga block")
    print(blockLong, blockShort)

    print("drawing items")
    # cv2.drawContours()
    print("returning anything")
    print("returning " + str(np.array([[rot_rect[0][0]],[rot_rect[0][1]]] )) + ", " + str(rotation))
    cv2.waitKey(0)
