import argparse
import imutils
import numpy as np
import cv2
import math
from time import sleep
from enum import Enum
import os
from shapely.geometry import Polygon, Point


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
    help="path to the input image")
args = vars(ap.parse_args())

class vision:
    def __init__(self):
        self.debug = False
        self.jengaDebug = False

        # Nathan Values for qt2
        # Exposure = 72
        #Brightness = 74
        #Contrast = 32
        # Saturation = 42
        # Gain = 50


        # Image Loading and Resizing
        self.needsBasis = True  # Run ceratin calculations if they haven't been run before
        if(os.name == 'posix'):
            self.camIndex = 2       # 0 is internal webcam
        else:
            self.camIndex = 0       # 0 is internal webcam
        self.image = None
        self.resized = None     #imutils.resize(image, width=self.resizedSize)
        self.drawImg = None
        self.resizedSize = 600
        self.ratio = None       #image.shape[0] / float(resized.shape[0])
        self.hsv = None         #cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

        # Create Masks Based on the Bounds
        self.blockMask = None

        self.mlDim = (100, 100) # machine-learning image size
        self.mlArray = []
        self.truthList = []
        self.imagePath = None

        # CONSTANTS
        # Offsets to use for all parts Only Using Two Squares RN
        self.blueWorldOrigin = np.array([[300],[400]])
        self.originTagPixel = None
        self.greenWorld = np.array([[-300],[400]])
        self.widthHeightLow = 0.8
        self.widthHeightHigh = 1.2
        self.expectedSize = 31   # TODO Change This in Pixels

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

        # Boundaries for the different  Colors
        #LINUX
        if(os.name == 'posix'):
            print("Running on Linux")
            self.lower_red = np.array([0,109,116])
            self.upper_red = np.array([27,255,255])

            self.lower_blue = np.array([80,90,107])
            self.upper_blue = np.array([118,218,255])

            self.lowerBlocks =  np.array([19,0,221])
            self.upperBlocks = np.array([112,49,255])

            self.lower_green = np.array([30,50,140])
            self.upper_green = np.array([86,170,255])
            self.lower_yellow = np.array([26,43,240])
            self.upper_yellow = np.array([43,110,255])
        # WINDOWS
        else:
            print("Running on Windows!")
            self.lower_red = np.array([0, 0, 0])
            self.upper_red = np.array([57, 255, 255])

            self.lower_blue = np.array([71,60,106])
            self.upper_blue = np.array([179,255,255])

            self.lowerBlocks =  np.array([19,0,200])
            self.upperBlocks = np.array([150,80,255])

            self.lower_green = np.array([28,38,144])
            self.upper_green = np.array([65,90,255])
            self.lower_yellow = np.array([26,43,240])
            self.upper_yellow = np.array([43,110,255])
        # Jenga block via pixels from our camera
        self.jengaWHigh = 49
        self.jengaWLow = 33
        self.jengaLHigh = 134
        self.jengaLLow = 115
        self.singleBlock = None     # Resets after finding a block sets to None if no block found

        self.basisWorld = None
        self.basisPixel = None
        self.frameRotation = 0

    def testCamera(self):
        # TODO: doesn't actually fail as instructed
        cap = cv2.VideoCapture(self.camIndex, cv2.CAP_DSHOW) # this is necessary for connecting to camera on Windows!
        # cap = cv2.VideoCapture(self.camIndex)
        if not cap.isOpened():
            cap.release()
            return False
        cap.release()
        return True

    def grabImage(self, delay = 2, fromPath = True):
        if fromPath:
            self.image = cv2.imread(args["image"])
        else:
            cam = cv2.VideoCapture(self.camIndex)
            sleep(delay)
            ret, self.image = cam.read()
        self.resized = imutils.resize(self.image, width=self.resizedSize)
        self.hsv = cv2.cvtColor(self.resized, cv2.COLOR_BGR2HSV)
        self.drawImg = self.resized.copy()
        if self.needsBasis:
            success = self.establishBasis2()
            print(f"Establish Basis Status? : {success}")
            if not success:
                return False
        # Always update the block mask
        self.blockMask = cv2.inRange(self.hsv, self.lowerBlocks, self.upperBlocks)
        if self.jengaDebug:
            cv2.imshow("HSV Block Mask", self.blockMask)  # Tag 1
            cv2.waitKey()
            cv2.imshow("HSV Block Mask", self.hsv)  # Tag 1
            cv2.waitKey()
        return True
    # TODO: WHY DOESNT THIS WORK!?!?!?
    #image = cv2.flip(image,0)


    def establishBasis2(self):
        self.ratio = self.image.shape[0] / float(self.resized.shape[0])
        gray = cv2.cvtColor(self.drawImg, cv2.COLOR_BGR2GRAY)

        # Create Masks Based on the Bounds

        blockMask = cv2.inRange(self.hsv, self.lowerBlocks, self.upperBlocks)
        if self.debug:
            cv2.imshow("HSV Block", blockMask) # Red Tag 2

        # gray_filtered = cv2.inRange(gray, 190, 255)
        gray_filtered = cv2.inRange(gray, 0, 190)

        if self.jengaDebug:
            cv2.imshow("testing 1", self.drawImg)
            cv2.imshow("testing gray 1", gray_filtered)
            cv2.waitKey(0)


        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray_filtered, arucoDict, parameters = arucoParams)
        print("(corners, ids, rejected)", (corners, ids, rejected))
        if len(corners) < 2:
            return False
        print("corners =", corners)
        print("corners[0] =", corners[0])
        # greenBox = np.array(corners[0])
        greenBox = tuple(corners)
        print("greenBox =", greenBox)
        print("type(greenBox) =", type(greenBox))

        self.drawContoursAruco(corners, ids)

        # print("corners, ", corners)
        # print("type of corners, ", type(corners))
        # print("corners[0], ", corners[0])
        # print("type of corners[0], ", type(corners[0]))
        # print("corners[0][0], ", corners[0][0])
        # print("type of corners[0][0], ", type(corners[0][0]))
        # print("corners[0][0][0], ", corners[0][0][0])
        # print("type of corners[0][0][0], ", type(corners[0][0][0]))
        # print("corners[0][0][0][0], ", corners[0][0][0][0])
        # print("type of corners[0][0][0][0], ", type(corners[0][0][0][0]))

        originCenter = np.mean(corners[1][0], axis = 0)
        print("greenCenter =", originCenter)
        secondaryCenter = np.mean(corners[0][0], axis = 0)
        print("redCenter =", secondaryCenter)
        # yellowCenter = np.mean(corners[0][0], axis = 0)
        # print("yellowCenter =", yellowCenter)
        # print("type(yellowCenter) =", type(yellowCenter))




        # Calculate the centers of each piece

        # (originCenter, blueBox) = self.getPixelCenterSquare(cntsBlue)
        # (secondaryCenter, greenBox) = self.getPixelCenterSquare(cntsGreen)

        if originCenter is None:
            print("Couldn't find center of blue Tag!")
            return False
        if secondaryCenter is None:
            return False
        #(greenCenter, greenBox) = self.getPixelCenterSquare(cntsGreen)

        # if greenBox is not None:
        #     self.drawContours(greenBox)
        #     self.drawPoint(greenCenter)

        self.drawPoint(originCenter, color = (255, 255, 255), r = 10)
        self.drawPoint(secondaryCenter)

        # Image coords pixels
        self.originTagPixel = np.array([[originCenter[0]], [originCenter[1]]])
        secondaryTag = np.array([[secondaryCenter[0]], [secondaryCenter[1]]])

        self.basisWorld, self.basisPixel, self.frameRotation = self.calcBasis(self.blueWorldOrigin, self.greenWorld, self.originTagPixel, secondaryTag)

        self.drawBasis()

        if self.debug:
            print(f"World Basis: \n{self.basisWorld}\nPixel Basis: \n{self.basisPixel}")

            # Testing the yellow block in center
            ####THESE PARAMS NEED TO BE FROM THE BLOCK/OPENCV
            blockRotationImage = 0
            blockCenterImageFullFrame = np.array([[originCenter[0]],
                                                [originCenter[1]]])
            print("yellow", originCenter)
            # Example to World Coords
            coordWorld, rotationWorld = self.changeBasisAtoB(self.blueWorldOrigin, self.originTagPixel, self.frameRotation, self.basisWorld,
                                                    blockCenterImageFullFrame, blockRotationImage )
            print(f"Yellow World Coords:\n {coordWorld}\nWorld Rotation: {rotationWorld}")
            # # Example to Pixel Coords
            testWorldCoords = np.array([[0],[0]])
            coordPixel, rotationBlock = self.changeBasisAtoB(self.originTagPixel, self.blueWorldOrigin , blockRotationImage, self.basisPixel,
                                                    testWorldCoords, self.frameRotation  )
            print(f"World->Pixel: {coordPixel}")
            self.drawPoint(coordPixel, (0,255,0), 5)
            for i in range(-300, 350, 50):
                for j in range(0, 500, 50):
                    testWorldCoords = np.array([[i],[j]])
                    coordPixel, rotationBlock = self.changeBasisAtoB(self.originTagPixel, self.blueWorldOrigin , blockRotationImage,
                                                    self.basisPixel, testWorldCoords, self.frameRotation  )
                    self.drawPoint(coordPixel, (0,0,255))
            cv2.imshow("ImageTest", self.drawImg)
            cv2.waitKey(0)
        self.drawImg = self.resized.copy()  # Reset the image for other operations!
        self.needsBasis = False
        return True # This means we were successful

    def establishBasis(self):
        self.ratio = self.image.shape[0] / float(self.resized.shape[0])
        self.hsv = cv2.cvtColor(self.resized, cv2.COLOR_BGR2HSV)

        # Create Masks Based on the Bounds
        greenMask = cv2.inRange(self.hsv, self.lower_green, self.upper_green)
        blueMask = cv2.inRange(self.hsv, self.lower_blue, self.upper_blue)
        redMask = cv2.inRange(self.hsv, self.lower_red, self.upper_red)
        blockMask = cv2.inRange(self.hsv, self.lowerBlocks, self.upperBlocks)
        if self.debug:
            cv2.imshow("HSV Green", greenMask)  # Tag 1
            cv2.imshow("HSV Blue", blueMask) # Yellow Tag 2
            cv2.imshow("HSV Red", redMask) # Red Tag 2
            cv2.imshow("HSV Block", blockMask) # Red Tag 2

        cntsBlue = self.getCnts(blueMask)
        cntsRed = self.getCnts(redMask)
        cntsGreen = self.getCnts(greenMask)

        if cntsGreen == [] or cntsRed == [] or cntsBlue == []:
            return False

        # Calculate the centers of each piece

        (blueCenter, blueBox) = self.getPixelCenterSquare(cntsBlue)
        (greenCenter, greenBox) = self.getPixelCenterSquare(cntsGreen)

        if blueCenter is None:
            print("Couldn't find center of blue Tag!")
            return False
        if greenCenter is None:
            return False
        #(greenCenter, greenBox) = self.getPixelCenterSquare(cntsGreen)

        # if greenBox is not None:
        #     self.drawContours(greenBox)
        #     self.drawPoint(greenCenter)
        if blueBox is not None:
            self.drawContours(blueBox)
            self.drawPoint(blueCenter)
        if greenBox is not None:
            self.drawContours(greenBox)
            self.drawPoint(greenCenter)

        # Image coords pixels
        self.originTagPixel = np.array([[blueCenter[0]], [blueCenter[1]]])
        secondaryTag = np.array([[greenCenter[0]], [greenCenter[1]]])

        self.basisWorld, self.basisPixel, self.frameRotation = self.calcBasis(self.blueWorldOrigin, self.greenWorld, self.originTagPixel, secondaryTag)

        self.drawBasis()

        if self.debug:
            print(f"World Basis: \n{self.basisWorld}\nPixel Basis: \n{self.basisPixel}")

            # Testing the yellow block in center
            ####THESE PARAMS NEED TO BE FROM THE BLOCK/OPENCV
            blockRotationImage = 0
            blockCenterImageFullFrame = np.array([[blueCenter[0]],
                                                [blueCenter[1]]])
            print("yellow", blueCenter)
            # Example to World Coords
            coordWorld, rotationWorld = self.changeBasisAtoB(self.blueWorldOrigin, self.originTagPixel, self.frameRotation, self.basisWorld,
                                                    blockCenterImageFullFrame, blockRotationImage )
            print(f"Yellow World Coords:\n {coordWorld}\nWorld Rotation: {rotationWorld}")
            # # Example to Pixel Coords
            testWorldCoords = np.array([[0],[0]])
            coordPixel, rotationBlock = self.changeBasisAtoB(self.originTagPixel, self.blueWorldOrigin , blockRotationImage, self.basisPixel,
                                                     testWorldCoords, self.frameRotation  )
            print(f"World->Pixel: {coordPixel}")
            self.drawPoint(coordPixel, (0,255,0), 5)
            for i in range(-300, 350, 50):
                for j in range(0, 500, 50):
                    testWorldCoords = np.array([[i],[j]])
                    coordPixel, rotationBlock = self.changeBasisAtoB(self.originTagPixel, self.blueWorldOrigin , blockRotationImage,
                                                    self.basisPixel, testWorldCoords, self.frameRotation  )
                    self.drawPoint(coordPixel, (0,0,255))
            cv2.imshow("ImageTest", self.drawImg)
            cv2.waitKey(0)
        self.drawImg = self.resized.copy()  # Reset the image for other operations!
        self.needsBasis = False
        return True # This means we were successful

    def getBlockPixel(self, mlFile = "C:/dev/delme/mlOutput.csv", truthFile = "C:/dev/delme/truthList.csv"):
        cnts = self.getCnts(self.blockMask)
        # print("cnts =", cnts)
        self.boxList = []
        self.boxPolygons = []
        rot_rectList = []
        rotationList = []
        self.truthList = []

        blockMask2 = cv2.cvtColor(self.blockMask, cv2.COLOR_GRAY2BGR) # add this line
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
            self.truthList.append(0)

            cv2.drawContours(blockMask2,[box], 0, self.outlineColorDict[classification.value], 2)
            # cv2.imshow("With Detection", blockMask2)

            # if drawOnOriginalImage:
            cv2.drawContours(self.drawImg,[box],0,self.outlineColorDict[classification.value], 2)
            cv2.imshow("Image", self.drawImg)

        self.pickBox(imgName = "Image 123123")
        print("truthlist =", self.truthList)
        print("file completed")


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
            print("Polygons =", self.boxPolygons)
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

        elif event == cv2.EVENT_RBUTTONDOWN:
            coordWorld, rotationWorld = self.changeBasisAtoB(self.greenWorldOrigin, self.originTagPixel, self.frameRotation, self.basisWorld, [x, y], 0)
            print("You just clicked on", coordWorld[0])
            print("The pixel coordinates are", (x, y))


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

    def getBlockPixelOld(self):
        blockPositions = []
        cnts = self.getCnts(self.blockMask)
        for c in reversed(cnts): # Start from top of frame since I can't seem to flip the image
            rot_rect = cv2.minAreaRect(c)
            rotation = rot_rect[2]
            # small pixel cutoff
            pixelCutoff = 10
            if rot_rect[1][1] < pixelCutoff or rot_rect[1][0] < pixelCutoff: # THIS HANDLES-> or rot_rect[1][1] == 0 or rot_rect[1][0] == 0:
                continue
            blockLong = max(rot_rect[1])*self.ratio
            blockShort = min(rot_rect[1])*self.ratio

            # use these for making sure block is right size!
            # print(blockLong)
            # print(blockShort)
            if blockLong > self.jengaLHigh or blockLong < self.jengaLLow or blockShort > self.jengaWHigh or blockShort < self.jengaWLow:
                # This isn't a jenga Block
                continue
                # TODO: LOOK FOR CLUSTERS HERE!!!

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

            blockPositions.append((np.array([[rot_rect[0][0]],[rot_rect[0][1]]] ), rotation))
            # return np.array([[rot_rect[0][0]],[rot_rect[0][1]]] ), rotation
            yield from blockPositions
        print("No blocks detected")
        return None

    def getBlockWorld(self):
        for singleBlockCenterPixel, singleBlockRotation in self.getBlockPixel():

            # Lets take the block into world Coordinates!!!
            coordWorld, rotationWorld = self.changeBasisAtoB(self.blueWorldOrigin, self.originTagPixel, self.frameRotation, self.basisWorld,
                                                    singleBlockCenterPixel, singleBlockRotation)

            # TODO: what does this return if nothing has happened??
            if self.jengaDebug:
                #print(f"Block: {singleBlockCenterPixel}, theta: {singleBlockRotation}")
                print(f"Jenga Block World Coords: {coordWorld}, World Rotation: {rotationWorld}")
                cv2.imshow("HSV Block", self.drawImg)
                cv2.waitKey(0)

            yield coordWorld, rotationWorld
            # return coordWorld, rotationWorld

    def drawContoursAruco(self, corners, ids):
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                        # draw the bounding box of the ArUCo detection
                cv2.line(self.drawImg, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(self.drawImg, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(self.drawImg, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(self.drawImg, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(self.drawImg, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(self.drawImg, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                # show the output image
                if self.jengaDebug:
                    cv2.imshow("Image 2", self.drawImg)
            cv2.waitKey(0)
    # loop over the contours
    def getPixelCenterSquare(self, cnts, expectedSize=90):
        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            rot_rect = cv2.minAreaRect(c)

            # This isn't Valid
            if rot_rect[1][0] == 0 or rot_rect[1][1] == 0:
                #print("there")
                continue

            # TODO Get Rid of too Big
            if rot_rect[1][0] < .8 * expectedSize or  rot_rect[1][1] < .8 * expectedSize:
                #print("Here")
                continue

            widthHeightRatio = rot_rect[1][0]/rot_rect[1][1]
            if widthHeightRatio > self.widthHeightHigh or widthHeightRatio < self.widthHeightLow:
                #Add a check to see if it about the right size!!
                # Tag bad size
                print("Bad tag size")
                continue

            #rotation = rot_rect[2]
            if self.debug:
                print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}")

            box = cv2.boxPoints(rot_rect) #* ratio
            box = np.int0(box)

            return (rot_rect[0], box) # Returns the center in pixels and also the box
        return (None, None) # Return empty tuple for recognition of a failure

    def calcBasis(self, baseTagWorld, secondaryTagWorld, baseTagPixel, secondaryTagPixel):
        """ np.array 2x1, np.array 2x1, np.array 2x1, np.array 2x1 """
        tagToTagVectorWorld = baseTagWorld - secondaryTagWorld
        distWorld = np.hypot(tagToTagVectorWorld[0], tagToTagVectorWorld[1]) # Calculates hypotenuse!

        tagToTagVectorPixel = baseTagPixel - secondaryTagPixel
        distPixel = np.hypot(tagToTagVectorPixel[0], tagToTagVectorPixel[1])

        frameRotation = math.atan2(tagToTagVectorPixel[1], tagToTagVectorPixel[0])  # This may need to be adjusted since images use weird coordinates

        mmPerPixel = distWorld/distPixel


        weirdCorrectionX = np.array([[1],[1]])
        weirdCorrectionY = np.array([[-1],[1]])
        xPosVector = 1* tagToTagVectorPixel/distPixel * mmPerPixel
        yPosVector = -1* np.vstack((xPosVector[1], xPosVector[0]))
        xPosVector = np.multiply(weirdCorrectionX, xPosVector)
        yPosVector = np.multiply(weirdCorrectionY, yPosVector)

        if(self.debug):
            print(f"THIS IS X\n{xPosVector}")
            print(f"THIS IS Y\n{yPosVector}")

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

    def tuneWindow(self):
        """
        Window used to tune HSV values this pops up if we can't find basis Tags
        """
        def nothing(x):
            pass
        cap = cv2.VideoCapture(self.camIndex)
        # Create a window
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        # create trackbars for color change
        cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
        cv2.createTrackbar('SMin','image',0,255,nothing)
        cv2.createTrackbar('VMin','image',0,255,nothing)
        cv2.createTrackbar('HMax','image',0,179,nothing)
        cv2.createTrackbar('SMax','image',0,255,nothing)
        cv2.createTrackbar('VMax','image',0,255,nothing)

        # Set default value for MAX HSV trackbars.
        cv2.setTrackbarPos('HMax', 'image', 179)
        cv2.setTrackbarPos('SMax', 'image', 255)
        cv2.setTrackbarPos('VMax', 'image', 255)

        # Initialize to check if HSV min/max value changes
        hMin = sMin = vMin = hMax = sMax = vMax = 0 # new values
        phMin = psMin = pvMin = phMax = psMax = pvMax = 0 # preious values

        # output = image
        wait_time = 33

        while True:
            ret, frame = cap.read()
            image = imutils.resize(frame, width=400)


            # get current positions of all trackbars
            hMin = cv2.getTrackbarPos('HMin','image')
            sMin = cv2.getTrackbarPos('SMin','image')
            vMin = cv2.getTrackbarPos('VMin','image')

            hMax = cv2.getTrackbarPos('HMax','image')
            sMax = cv2.getTrackbarPos('SMax','image')
            vMax = cv2.getTrackbarPos('VMax','image')

            # Set minimum and max HSV values to display
            lower = np.array([hMin, sMin, vMin])
            upper = np.array([hMax, sMax, vMax])

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            inverseHue = hMin > hMax
            # account for hue wrapping around 255
            if inverseHue:
                print("inverseHue is in effect!")
                inverseLower1 = np.array([0, sMin, vMin])
                inverseUpper1 = np.array([hMax, sMax, vMax])
                inverseLower2 = np.array([hMin, sMin, vMin])
                inverseUpper2 = np.array([255, sMax, vMax])

                mask = cv2.inRange(hsv, inverseLower1, inverseUpper1) | cv2.inRange(hsv, inverseLower2, inverseUpper2)
            else:
                # Create HSV Image and threshold into a range.
                mask = cv2.inRange(hsv, lower, upper)

            output = cv2.bitwise_and(image,image, mask= mask)

            # Print if there is a change in HSV value
            if( (phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
                print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
                phMin = hMin
                psMin = sMin
                pvMin = vMin
                phMax = hMax
                psMax = sMax
                pvMax = vMax

            # Display output image
            if self.jengaDebug:
                cv2.imshow('image',output)

            # Wait longer to prevent freeze for videos.
            if cv2.waitKey(wait_time) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

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
