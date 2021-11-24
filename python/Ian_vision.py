# Call with
# python3 Ian_vision.py -i C:\dev\git\armadillo\cvTesting\test_frames\frame_7.png

import argparse
import imutils
import numpy as np
import cv2
import math
from ColorPicker import ColorPicker
from time import sleep
import json

from numpy.core.shape_base import block
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
    help="path to the input image")
args = vars(ap.parse_args())

class vision:
    def __init__(self):
        self.debug = True
        self.jengaDebug = True

        # Image Loading and Resizing
        self.needsBasis = True  # Run ceratin calculations if they haven't been run before
        self.camIndex = 2       # 0 is internal webcam
        self.image = None
        self.resized = None     #imutils.resize(image, width=self.resizedSize)
        self.drawImg = None
        self.resizedSize = 1000
        self.ratio = None       #image.shape[0] / float(resized.shape[0])
        self.hsv = None         #cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

        # Create Masks Based on the Bounds
        self.blockMask = None

        # CONSTANTS
        # Offsets to use for all parts Only Using Two Squares RN
        self.greenWorldOrigin = np.array([[200],[525]])
        self.originTagPixel = None
        self.redWorld = np.array([[-200],[525]])
        self.widthHeightLow = 0.9
        self.widthHeightHigh = 1.1
        self.expectedSize = 31   # TODO Change This in Pixels

        # Boundaries for the different  Colors
        self.lower_red = np.array([0,150,180])
        self.upper_red = np.array([22,240,240])
        self.lower_green = np.array([66,82,77])
        self.upper_green = np.array([87,208,190])

        self.lower_yellow = np.array([26,43,240])
        self.upper_yellow = np.array([43,110,255])

        self.lowerBlocks =  np.array([10,0,140])
        self.upperBlocks = np.array([47,70,255])

        # Jenga block via pixels from our camera
        self.jengaWHigh = 42
        self.jengaWLow = 36
        self.jengaLHigh = 121
        self.jengaLLow = 115

        # Jenga block sizes under dilated image
        self.jengaWHighDilated = 100
        self.jengaWLowDilated = 5
        self.jengaLHighDilated = 190
        self.jengaLLowDilated = 5

        self.singleBlock = None     # Resets after finding a block sets to None if no block found

        self.basisWorld = None
        self.basisPixel = None
        self.frameRotation = 0

    def testCamera(self):
        # TODO: doesn't actually fail as instructed
        cap = cv2.VideoCapture(self.camIndex)
        # Check if the webcam is opened correctly
        # while True:
        #     ret, frame = cap.read()
        #     if ret == True:
        #         cv2.imshow('Frame',frame)
        #         # Press Q on keyboard to  exit
        #         if self.checkWaitKey(25) & 0xFF == ord('q'):
        #             break


        if not cap.isOpened():
            cap.release()
            return False
        cap.release()
        return True

    def pickSwatchColors(self, filename = "C:/dev/delme/colorPickerTest.txt"):
        print("picking swatch colors")
        # filename contains the default color choices

        f = open(filename, "r+")
        print("opened file successfully")
        contents = str(f.read())
        print(contents)
        oldSwatches = json.loads(contents)
        newSwatches = oldSwatches.copy() # initialize using the existing values

        cp = ColorPicker(self.drawImg)

        swatchesToPick = ["red", "green", "yellow", "block"]
        print(swatchesToPick)
        for swatchColor in swatchesToPick:
            print(f"Pick the {swatchColor} swatch")
            myColor = cp.colorPicker()
            if myColor is not None:
                newSwatches[swatchColor] = myColor
            else: # if the ESC key was pressed
        # else: it stays the same as the oldSwatches; stop picking any more colors
                break

        if newSwatches != oldSwatches:
            print(newSwatches)
            f.truncate(0) # delete contents
            f.seek(0) # move cursor to initial position
            f.write(json.dumps({key: [val2.item() for val2 in val] for key, val in newSwatches.items()}))
        f.close()

        offset = np.array([40, 40, 40])
        yellowOffset = np.array([10, 30, 30])

        val_red = np.array(newSwatches["red"])
        val_green = np.array(newSwatches["green"])
        val_yellow = np.array(newSwatches["yellow"])
        val_block = np.array(newSwatches["block"])

        # reset boundary colors
        self.lower_red = val_red - offset
        self.upper_red = val_red + offset
        self.lower_green = val_green - offset
        self.upper_green = val_green + offset
        self.lower_yellow = val_yellow - yellowOffset
        self.upper_yellow = val_yellow + yellowOffset

        self.lowerBlocks = val_block - offset
        self.upperBlocks = val_block + offset


    def grabImage(self, delay = 2, fromPath = True):
        if fromPath:
            self.image = cv2.imread(args["image"])
        else:
            cam = cv2.VideoCapture(self.camIndex)
            sleep(delay)
            ret, self.image = cam.read()
        self.resized = imutils.resize(self.image, width=self.resizedSize)
        self.drawImg = self.resized.copy()

        vs.pickSwatchColors() # reset the color basis for the swatches

        if self.needsBasis:
            success = self.establishBasis()
            if not success:
                return False
        # Always update the block mask

        self.blockMask = cv2.inRange(self.hsv, self.lowerBlocks, self.upperBlocks)

        kernel = np.ones((4,4), np.uint8)
        eroded = cv2.erode(self.blockMask, kernel, iterations=3)
        cv2.imshow("eroded", eroded)
        self.blockMask = eroded

        print(self.blockMask)

        if self.jengaDebug:
            cv2.imshow("HSV Block Mask", self.blockMask)  # Tag 1
        return True
    # TODO: WHY DOESNT THIS WORK!?!?!?
    #image = cv2.flip(image,0)

    def establishBasis(self):
        self.ratio = self.image.shape[0] / float(self.resized.shape[0])
        self.hsv = cv2.cvtColor(self.resized, cv2.COLOR_BGR2HSV)

        # Create Masks Based on the Bounds
        greenMask = cv2.inRange(self.hsv, self.lower_green, self.upper_green)
        yellowMask = cv2.inRange(self.hsv, self.lower_yellow, self.upper_yellow)
        redMask = cv2.inRange(self.hsv, self.lower_red, self.upper_red)

        if self.debug:
            cv2.imshow("HSV Green Before", greenMask)  # Tag 1
            cv2.imshow("HSV Yellow Before", yellowMask) # Yellow Tag 2
            cv2.imshow("HSV Red Before", redMask) # Red Tag 2

        kernel = np.ones((8,8), np.uint8) * 2
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel, iterations = 2)
        redMask = cv2.morphologyEx(redMask, cv2.MORPH_OPEN, kernel, iterations = 2)
        greenMask = cv2.morphologyEx(greenMask, cv2.MORPH_OPEN, kernel, iterations = 2)

        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernel, iterations = 5)
        redMask = cv2.morphologyEx(redMask, cv2.MORPH_CLOSE, kernel, iterations = 5)
        greenMask = cv2.morphologyEx(greenMask, cv2.MORPH_CLOSE, kernel, iterations = 5)

        if self.debug:
            cv2.imshow("HSV Green", greenMask)  # Tag 1
            cv2.imshow("HSV Yellow", yellowMask) # Yellow Tag 2
            cv2.imshow("HSV Red", redMask) # Red Tag 2


        self.checkWaitKey()

        cntsYellow = self.getCnts(yellowMask)
        # print("cntsYellow: ")
        # print(cntsYellow)
        cntsRed = self.getCnts(redMask)
        # print("cntsRed: ")
        # print(cntsRed)
        cntsGreen = self.getCnts(greenMask)
        # print("cntsGreen: ")
        # print(cntsGreen)

        if not cntsGreen or not cntsRed:
            return False

        # Calculate the centers of each piece
        (greenCenter, greenBox) = self.getPixelCenterSquare(cntsGreen)
        (yellowCenter, yellowBox) = self.getPixelCenterSquare(cntsYellow)
        (redCenter, redBox) = self.getPixelCenterSquare(cntsRed)

        if greenBox is not None:
            self.drawContours(greenBox)
            self.drawPoint(greenCenter)
        if yellowBox is not None:
            self.drawContours(yellowBox)
            self.drawPoint(yellowCenter)
        if redBox is not None:
            self.drawContours(redBox)
            self.drawPoint(redCenter)


        # Image coords pixels
        self.originTagPixel = np.array([[greenCenter[0]], [greenCenter[1]]])
        secondaryTag = np.array([[redCenter[0]], [redCenter[1]]])

        self.basisWorld, self.basisPixel, self.frameRotation = self.calcBasis(self.greenWorldOrigin, self.redWorld, self.originTagPixel, secondaryTag)

        self.drawBasis()

        if self.debug:
            print(f"World Basis: \n{self.basisWorld}\nPixel Basis: \n{self.basisPixel}")

            # Testing the yellow block in center
            ####THESE PARAMS NEED TO BE FROM THE BLOCK/OPENCV
            blockRotationImage = 0
            blockCenterImageFullFrame = np.array([[yellowCenter[0]],
                                                [yellowCenter[1]]])
            print("yellow", yellowCenter)
            # Example to World Coords
            coordWorld, rotationWorld = self.changeBasisAtoB(self.greenWorldOrigin, self.originTagPixel, self.frameRotation, self.basisWorld,
                                                    blockCenterImageFullFrame, blockRotationImage )
            print(f"Yellow World Coords:\n {coordWorld}\nWorld Rotation: {rotationWorld}")
            # # Example to Pixel Coords
            testWorldCoords = np.array([[0],[100]])
            coordPixel, rotationBlock = self.changeBasisAtoB(self.originTagPixel, self.greenWorldOrigin , blockRotationImage, self.basisPixel,
                                                     testWorldCoords, self.frameRotation  )
            print(f"World->Pixel: {coordPixel}")
            self.drawPoint(coordPixel, (0,255,0), 5)
            for i in range(-500, 500, 50):
                for j in range(0, 650, 50):
                    testWorldCoords = np.array([[i],[j]])
                    coordPixel, rotationBlock = self.changeBasisAtoB(self.originTagPixel, self.greenWorldOrigin , blockRotationImage,
                                                    self.basisPixel, testWorldCoords, self.frameRotation  )
                    self.drawPoint(coordPixel, (0,0,255))
            cv2.imshow("Image", self.drawImg)
            self.checkWaitKey(0)
        self.drawImg = self.resized.copy()  # Reset the image for other operations!
        self.needsBasis = False
        return True # This means we were successful

    # Wrap some basic OpenCV for Easier Use
    def drawContours(self, box, color=(0, 0, 255), thickness=2):
        cv2.drawContours(self.drawImg,[box],0,(0, 0, 255),thickness)

    def drawPoint(self, point, color = (0, 255, 255), r = 1):
        cv2.circle(self.drawImg, (int(point[0]), int(point[1])), r, color, 5)

    def drawBasis(self, arrowSize = 20, colorX = (0, 255, 0), colorY = (0, 0, 255)):
        originTuple = (int(self.originTagPixel[0]), int(self.originTagPixel[1]))

        basisLarger = arrowSize*self.basisWorld + self.originTagPixel
        endXPosVec = (int(basisLarger[0][0]), int(basisLarger[1][0]))
        endYPosVec = (int(basisLarger[0][1]), int(basisLarger[1][1]))
        cv2.arrowedLine(self.drawImg, originTuple, endXPosVec, colorX, 2)
        cv2.arrowedLine(self.drawImg, originTuple, endYPosVec, colorY, 2)

    def getCnts(self, mask):
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
        return imutils.grab_contours(cnts)

    def getBlockPixel(self):
        cnts = self.getCnts(self.blockMask)
        print(cnts)
        blockMask2 = cv2.cvtColor(self.blockMask, cv2.COLOR_GRAY2BGR)  #add this line
        for c in reversed(cnts): # Start from top of frame since I can't seem to flip the image
            rot_rect = cv2.minAreaRect(c)
            rotation = rot_rect[2]
            # small pixel cutoff
            pixelCutoff = 1
            if rot_rect[1][1] < pixelCutoff or rot_rect[1][0] < pixelCutoff: # THIS HANDLES-> or rot_rect[1][1] == 0 or rot_rect[1][0] == 0:
                continue
            blockLong = max(rot_rect[1])*self.ratio
            blockShort = min(rot_rect[1])*self.ratio


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

                criteria1 = blockLong < 110
                criteria2 = blockLong > 60
                criteria3 = blockShort < 25
                criteria4 = blockShort > 10

                if not (criteria1 and criteria2 and criteria3 and criteria4):
                    # This isn't a jenga Block
                    print("this isn't a jenga block")
                    print(blockLong, blockShort)
                    cv2.drawContours(blockMask2,[box],0,(255, 0, 0), 2)
                    cv2.imshow("With Detection", blockMask2)
                    self.checkWaitKey(0)
                    continue
                    # TODO: LOOK FOR CLUSTERS HERE!!!



                self.drawContours(box, thickness = 6)
                self.drawPoint(rot_rect[0], (255,0,0))
                # cv2.drawContours()
                cv2.drawContours(blockMask2,[box],0,(0, 255, 255), 2)
                cv2.imshow("With Detection", blockMask2)
                cv2.drawContours(self.drawImg,[box],0,(0, 255, 255), 2)
                cv2.imshow("Image", self.drawImg)
                self.checkWaitKey(0)
                # return np.array([[rot_rect[0][0]],[rot_rect[0][1]]] ), rotation

        return None

    def getBlockWorld(self):
        singleBlockCenterPixel, singleBlockRotation =  self.getBlockPixel()
        # Lets take the block into world Coordinates!!!
        coordWorld, rotationWorld = self.changeBasisAtoB(self.greenWorldOrigin, self.originTagPixel, self.frameRotation, self.basisWorld,
                                                singleBlockCenterPixel, singleBlockRotation)

        # TODO: what does this return if nothing has happened??
        if self.jengaDebug:
            #print(f"Block: {singleBlockCenterPixel}, theta: {singleBlockRotation}")
            print(f"Jenga Block World Coords: {coordWorld}, World Rotation: {rotationWorld}")
            cv2.imshow("HSV Block", self.drawImg)
            self.checkWaitKey(0)
        return coordWorld, rotationWorld

    # loop over the contours
    def getPixelCenterSquare(self, cnts):
        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            rot_rect = cv2.minAreaRect(c)

            # This isn't Valid
            if rot_rect[1][0] == 0 or rot_rect[1][1] == 0:
                continue

            # TODO Get Rid of too Big
            #if rot_rect[1][0] > 1.2 * expectedSize or rot_rect[1][0] < .8 * expectedSize or  rot_rect[1][1] > 1.2 * expectedSize or rot_rect[1][1] < .8 * expectedSize:
            #    continue

            widthHeightRatio = rot_rect[1][0]/rot_rect[1][1]
            if widthHeightRatio > self.widthHeightHigh or widthHeightRatio < self.widthHeightLow:
                #Add a check to see if it about the right size!!
                continue

            #rotation = rot_rect[2]
            if self.debug:
                print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}")

            box = cv2.boxPoints(rot_rect) #* ratio
            box = np.int0(box)

            return (rot_rect[0], box) # Returns the center in pixels and also the box

    def calcBasis(self, baseTagWorld, secondaryTagWorld, baseTagPixel, secondaryTagPixel):
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

    def changeBasisAtoB(self, bOffset, aOffset, fRot, basis, aVector, bRot):
        """
        Expects np.array(mm), np.array(pixels), radians, np.array(2x2)(mm), np.array(pixels), radians
        """
        blockCenterImage = aVector - aOffset
        bVector = np.matmul(basis, blockCenterImage)
        bVector = bVector + bOffset
        rotation = bRot-fRot
        return bVector, rotation

    def checkWaitKey(self, val=0): # val = 0 displays static frame; val = 1 dispays moving frame
        # perform the waitKey, then check key strokes for special actions
        key = cv2.waitKey(val) & 0xFF
        if key == 27: # ESC
            cv2.destroyAllWindows()
            exit()
        if key == 32: # Space
            print("You pressed space!")


if __name__=='__main__':
    # arm = robot.robot()
    vs = vision()

    # testingHomingandWorld = True
    testingCameras = True

    """
    COde to Detect basis and camera tags
    """
    if testingCameras:
        if not vs.testCamera():
            print("Camera Not working")

        grabbingFrame = True
        # while grabbingFrame:
        grabImageSuccess = vs.grabImage(fromPath=True)

        if not grabImageSuccess:
            print("Please reposition Camera and check masking!")
            vs.tuneWindow()
            x = input('Retry (R) or Quit (Q): ')
            if x == 'R':
                pass
            else:
                quit()

        vs.getBlockWorld()