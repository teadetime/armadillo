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
from enum import Enum
from scipy import ndimage
import pandas as pd
from shapely.geometry import Polygon, Point


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
        self.camIndex = 0       # 0 is internal webcam
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

        self.outlineColorDict = { # BGR
            self.BlockType.NOT_BLOCK.value: (140, 0, 0), # dark blue
            self.BlockType.BLOCK.value: (0, 255, 255), # yellow
            self.BlockType.CLUSTER.value: (0, 255, 0), # green
            self.BlockType.EDGE_BLOCK.value: (0, 180, 180), # dark yellow
            self.BlockType.END_BLOCK.value: (0, 128, 128), # darker yellow
            self.BlockType.OUT_OF_BOUNDS.value: (0, 0, 0), # black
            self.BlockType.SWATCH.value: (0, 0, 255), # red
            self.BlockType.ROBOT.value: (0, 0, 170), # dark red
        }

        self.mlDim = (100, 100) # machine-learning image size
        self.mlArray = []
        self.truthList = []
        self.imagePath = None

        self.singleBlock = None     # Resets after finding a block sets to None if no block found

        self.basisWorld = None
        self.basisPixel = None
        self.frameRotation = 0

    def testCamera(self):
        # TODO: doesn't actually fail as instructed
        cap = cv2.VideoCapture(self.camIndex, cv2.CAP_DSHOW)
        print(cap)
        # Check if the webcam is opened correctly
        # while True:
        #     ret, frame = cap.read()
        #     if ret == True:
        #         cv2.imshow('Frame',frame)
        #         # Press Q on keyboard to  exit
        #         if cv2.waitKey(25) & 0xFF == ord('q'):
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
        try:
            oldSwatches = json.loads(contents)
        except:
            print("please select all colors manually")
            oldSwatches = {}
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
        else:
            print("Using swatches previously stored. Thank you!")
        f.close()

        offset = np.array([60, 60, 60])
        redOffset = np.array([30, 60, 60])
        yellowOffset = np.array([50, 30, 10])

        val_red = np.array(newSwatches["red"])
        val_green = np.array(newSwatches["green"])
        val_yellow = np.array(newSwatches["yellow"])
        val_block = np.array(newSwatches["block"])

        # reset boundary colors
        self.lower_red = val_red - redOffset
        self.upper_red = val_red + redOffset
        self.lower_green = val_green - offset
        self.upper_green = val_green + offset
        self.lower_yellow = val_yellow - yellowOffset
        self.upper_yellow = val_yellow + yellowOffset

        self.lowerBlocks = val_block - offset
        self.upperBlocks = val_block + offset


    def grabImage(self, delay = 2, fromPath = True, path = None, pickSwatches = True):
        if path is not None:
            self.image = cv2.imread(path)
            self.imagePath = path
        elif fromPath:
            self.image = cv2.imread(args["image"])
            self.imagePath = args["image"]
        else:
            cam = cv2.VideoCapture(self.camIndex)
            sleep(delay)
            ret, self.image = cam.read()

        self.resized = imutils.resize(self.image, width=self.resizedSize)
        self.drawImg = self.resized.copy()

        if pickSwatches:
            self.pickSwatchColors() # reset the color basis for the swatches

        if self.needsBasis:
            success = self.establishBasis()
            if not success:
                return False
        # Always update the block mask

        self.blockMask = cv2.inRange(self.hsv, self.lowerBlocks, self.upperBlocks)

        kernel = np.ones((3,3), np.uint8)
        eroded = cv2.erode(self.blockMask, kernel, iterations=3)
        cv2.imshow("eroded", eroded)
        self.blockMask = eroded

        print(self.blockMask)

        if self.jengaDebug:
            cv2.imshow("HSV Block Mask", self.blockMask)  # Tag 1
        return True
    # TODO: WHY DOESNT THIS WORK!?!?!?
    #image = cv2.flip(image,0)

    def establishBasisAruco(self):
        self.ratio = self.image.shape[0] / float(self.resized.shape[0])

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.drawImg, arucoDict, parameters = arucoParams)
        print("(corners, ids, rejected)", (corners, ids, rejected))

        # Calculate the centers of each piece
        (greenCenter, greenBox) = self.getPixelCenterSquare(cntsGreen)
        (yellowCenter, yellowBox) = self.getPixelCenterSquare(cntsYellow)
        (redCenter, redBox) = self.getPixelCenterSquare(cntsRed)

        greenBox = corners[0]
        print(greenBox)

        greenCenter = np.mean(corners[0], axis = 0)
        redCenter = np.mean(corners[1], axis = 0)
        greenCenter = np.mean(corners[0], axis = 0)

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

        kernel = np.ones((5,5), np.uint8) * 2

        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernel, iterations = 4)
        redMask = cv2.morphologyEx(redMask, cv2.MORPH_CLOSE, kernel, iterations = 4)
        greenMask = cv2.morphologyEx(greenMask, cv2.MORPH_CLOSE, kernel, iterations = 4)

        # yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel, iterations = 4)
        # redMask = cv2.morphologyEx(redMask, cv2.MORPH_OPEN, kernel, iterations = 4)
        # greenMask = cv2.morphologyEx(greenMask, cv2.MORPH_OPEN, kernel, iterations = 4)

        # yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernel, iterations = 4)
        # redMask = cv2.morphologyEx(redMask, cv2.MORPH_CLOSE, kernel, iterations = 4)
        # greenMask = cv2.morphologyEx(greenMask, cv2.MORPH_CLOSE, kernel, iterations = 4)

        if self.debug:
            cv2.imshow("HSV Green", greenMask)  # Tag 1
            cv2.imshow("HSV Yellow", yellowMask) # Yellow Tag 2
            cv2.imshow("HSV Red", redMask) # Red Tag 2


        self.checkWaitKey()

        cntsYellow = self.getCnts(yellowMask)
        cntsRed = self.getCnts(redMask)
        cntsGreen = self.getCnts(greenMask)

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

    def getBlockPixel(self, mlFile = "C:/dev/delme/mlOutput.csv", truthFile = "C:/dev/delme/truthList.csv"):
        cnts = self.getCnts(self.blockMask)
        print(cnts)
        self.boxList = []
        self.boxPolygons = []
        rot_rectList = []
        rotationList = []
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

            rot_rectList.append(rot_rect)
            rotationList.append(rotation)

            #DEBUG
            if self.jengaDebug:
                print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}, rotation: {rotation}")
                print(f"ratio:{self.ratio}, w: {self.ratio*rot_rect[1][0]}, h: {self.ratio*rot_rect[1][1]}")
                print(f"Final rotation: {rotation}")

                box = cv2.boxPoints(rot_rect) #* ratio
                box = np.int0(box)

                classification = self.checkBlockCriteria(blockShort, blockLong, box, rotation)
                if classification == self.BlockType.OUT_OF_BOUNDS:
                    continue

                self.boxList.append(box)
                self.boxPolygons.append(Polygon(box))

                # if classification == self.BlockType.NOT_BLOCK:
                #     color = (255, 0, 0)
                #     drawOnOriginalImage = False
                # elif classification == self.BlockType.CLUSTER:
                #     color = (0, 255, 0)
                #     drawOnOriginalImage = False
                # elif classification == self.BlockType.BLOCK:
                #     color = (0, 255, 255)
                #     drawOnOriginalImage = True

                cv2.drawContours(blockMask2,[box], 0, self.outlineColorDict[classification.value], 2)
                cv2.imshow("With Detection", blockMask2)

                # if drawOnOriginalImage:
                cv2.drawContours(self.drawImg,[box],0,self.outlineColorDict[classification.value], 2)
                cv2.imshow("Image", self.drawImg)

                # newClassification = self.receiveClassificationInput(default = classification)
                # self.truthList.append(newClassification)
                self.truthList.append(classification.value)

        self.pickBox(imgName = "Image")
        # pd.DataFrame(np.array(self.mlArray)).to_csv("C:/dev/delme/mlOutput2.csv")
        print("mlArray[0] =", np.array(self.mlArray)[0].shape)
        print("mlArray[0] =", self.mlArray[0])
        print(type(self.mlArray))
        arrayToDisplay = np.stack(self.mlArray, axis = 0)
        print("size of mlArray:", arrayToDisplay.shape)
        print("truthlist =", self.truthList)
        # arrayToDisplay.tofile("C:/dev/delme/mlOutput2.csv", format = "%1.2f")
        np.savetxt(mlFile, arrayToDisplay, delimiter = ",", fmt='%f')
        np.savetxt(truthFile, self.truthList, delimiter = ",", fmt='%f')
        print("file completed")
        self.checkWaitKey(0)

    # return np.array([[rot_rect[0][0]],[rot_rect[0][1]]] ), rotation


        for this_classification, this_rot_rect, this_rotation in zip(self.truthList, rot_rectList, rotationList):
            if this_classification == 1:
                yield np.array([[this_rot_rect[0][0]],[rot_rect[0][1]]] ), this_rotation
        # return None

    def receiveClassificationInput(self, default = None):
        newClassification = self.checkWaitKey(0) & 0xFF
        if newClassification in set( ord(str(item.value)) for item in self.BlockType ):
            return newClassification - ord('0')
        else:
            return default

    class BlockType(Enum):
        NOT_BLOCK = 0
        BLOCK = 1
        CLUSTER = 2
        EDGE_BLOCK = 3
        END_BLOCK = 4
        OUT_OF_BOUNDS = 5
        SWATCH = 6
        ROBOT = 7

    def pickBox(self, imgName):
        self.boxPickImgName = imgName
        cv2.namedWindow(imgName)

        #mouse call back function declaration
        cv2.setMouseCallback(imgName, self.selectBox)

        print("Left click to choose a color, spacebar to save and exit")

        #while loop to live update
        while (1):
            cv2.imshow(imgName, self.drawImg)
            if self.checkWaitKey(1) == ord(' '):
                break

# Mouse Callback function - this is triggered every time the mouse moves
    def selectBox(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for i, boxPolygon in enumerate(self.boxPolygons):
                if boxPolygon.contains(Point(x, y)):
                    cv2.drawContours(self.drawImg,[self.boxList[i]],0,(255, 255, 0), 2) # modifying color: cyan
                    cv2.imshow(self.boxPickImgName, self.drawImg)
                    classification = self.receiveClassificationInput()
                    print("classification = ", classification)
                    if classification is not None: # if the default value None was not returned:
                        print("truthList =", self.truthList)
                        print("len =", len(self.truthList))
                        print("i =", i)
                        self.truthList[i] = classification
                        cv2.drawContours(self.drawImg, [self.boxList[i]], 0, self.outlineColorDict[classification], 2) # modified color: magenta
                        cv2.imshow(self.boxPickImgName, self.drawImg)

    def checkBlockCriteria(self, blockShort, blockLong, box, rotation):

        print("Box = ")
        print(box)
        x1 = min(box, key = lambda x: x[0])[0]
        x2 = max(box, key = lambda x: x[0])[0]
        y1 = min(box, key = lambda x: x[1])[1]
        y2 = max(box, key = lambda x: x[1])[1]
        print(x1, x2, y1, y2)
        closeUp = self.blockMask[y1:y2, x1:x2]
        print("rotation = ", rotation)
        print(closeUp)
        if closeUp.size <= 0: # check if the image is empty
            return self.BlockType.OUT_OF_BOUNDS

        cv2.imshow("Box Close-Up", closeUp)
        # self.checkWaitKey(0)
        #rotation angle in degree
        rotated = ndimage.rotate(closeUp, -rotation)
        cv2.imshow("Rotated Close-Up", rotated)

        resized = self.basicResize(rotated, self.mlDim)
        cv2.imshow("Scaled Rotated Close-Up", resized)

        print("image variable type =", type(resized))
        newVector = np.reshape(resized, -1) # convert to a vector
        print("newVector shape =", newVector.shape)

        self.mlArray.append(newVector)

        if self.jengaDebug:
            print(blockLong, blockShort)

        inBlockDimensionRange = blockLong  > 60 and blockLong  < 110 and \
                                blockShort > 10 and blockShort < 25

        if inBlockDimensionRange:
            return self.BlockType.BLOCK

        inClusterDimensionRange = blockLong  > 60 and blockLong  < 150 and \
                                  blockShort > 20 and blockShort < 100

        if inClusterDimensionRange:
            return self.BlockType.CLUSTER

        return self.BlockType.NOT_BLOCK

    def getBlockWorld(self):
      for singleBlockCenterPixel, singleBlockRotation in self.getBlockPixel(): # reorder this code once it works i.e. don't call the change basis function on every iteration
        # singleBlockCenterPixel, singleBlockRotation = next(self.getBlockPixel())

        # Lets take the block into world Coordinates!!!
        coordWorld, rotationWorld = self.changeBasisAtoB(self.greenWorldOrigin, self.originTagPixel, self.frameRotation, self.basisWorld,
                                                singleBlockCenterPixel, singleBlockRotation)

        # TODO: what does this return if nothing has happened??
        if self.jengaDebug and 0:
            #print(f"Block: {singleBlockCenterPixel}, theta: {singleBlockRotation}")
            print(f"Jenga Block World Coords: {coordWorld}, World Rotation: {rotationWorld}")
            cv2.imshow("HSV Block", self.drawImg)
            self.checkWaitKey(0)
        # return coordWorld, rotationWorld
        yield coordWorld, rotationWorld

    # loop over the contours
    def getPixelCenterSquare(self, cnts):
        # self.checkWaitKey()
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

    def basicResize(self, image, newDim):
        oldDim = image.shape
        image2 = image.copy()

        # First crop the image
        if newDim[0] < oldDim[0]:
            x1 = int(np.floor(           (oldDim[0] - newDim[0]) / 2))
            x2 = int(np.floor(oldDim[0] - (oldDim[0] - newDim[0]) / 2)) # off by one error
        else:
            x1 = 0
            x2 = oldDim[0]

        if newDim[1] < oldDim[1]:
            y1 = int(np.floor(           (oldDim[1] - newDim[1]) / 2))
            y2 = int(np.floor(oldDim[1] - (oldDim[1] - newDim[1]) / 2))
        else:
            y1 = 0
            y2 = oldDim[1]

        image2 = image2[x1:x2, y1:y2] # crop image
        ##--------##

        # Then add borders as necessary
        if newDim[0] > oldDim[0]:
            y1padding = int(np.floor((newDim[0] - oldDim[0]) / 2))
            y2padding = int( np.ceil((newDim[0] - oldDim[0]) / 2))
        else:
            y1padding = 0
            y2padding = 0

        if newDim[1] > oldDim[1]:
            x1padding = int(np.floor((newDim[1] - oldDim[1]) / 2))
            x2padding = int( np.ceil((newDim[1] - oldDim[1]) / 2))
        else:
            x1padding = 0
            x2padding = 0

        print("padding y1, y2, x1, x2 =", y1padding, y2padding, x1padding, x2padding)
        image2 = cv2.copyMakeBorder(image2, y1padding, y2padding, x1padding, x2padding, cv2.BORDER_CONSTANT, value = 0)

        return image2

    def checkWaitKey(self, val=0): # val = 0 displays static frame; val = 1 dispays moving frame
        # perform the waitKey, then check key strokes for special actions
        key = cv2.waitKey(val) & 0xFF
        if key == 27: # ESC
            cv2.destroyAllWindows()
            exit()
        if key == 32: # Space
            pass # could put debugging tools or something here
            # TODO: toggle calibration flag and/or send calibration signal
        return key


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