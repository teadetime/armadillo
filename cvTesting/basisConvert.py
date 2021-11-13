# Lets look at basis transform of coordinates
import numpy as np
from numpy.core.shape_base import hstack
import math
# Offsets to use for all parts
tag1World = np.array([[300],[200]])
tag2World = np.array([[500],[100]])
offset = tag1World-tag2World
distWorld = np.hypot(offset[0], offset[1] ) # Calculates hypotenuse!
print(distWorld)


# Recognize tag1
# Image coords pixels
u1 = 30
v1 = 40

tag1Center = np.array([[u1], 
                      [v1]])


# Image coords pixels
u2 = 120
v2 = 40
tag2Center = np.array([[u2], 
                      [v2]])

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
blockRotationImage = 80
blockCenterImageFullFrame = np.array([[30],
                                      [40]])

blockCenterImage = blockCenterImageFullFrame - tag1Center

coordinatesWorld = np.matmul(basisWorld, blockCenterImage)
coordinatesWorld = coordinatesWorld + tag1World
rotationWorld = blockRotationImage-frameRotation


print(f"Coords: {coordinatesWorld}, Rotation: {rotationWorld}")