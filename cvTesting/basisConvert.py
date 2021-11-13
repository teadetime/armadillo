# Lets look at basis transform of coordinates
import numpy as np

# Offsets to use for all parts
tag1World = np.array([[300],[200]])
tag2World = np.array([[500],[100]])
offset = tag1World-tag2World
dist = np.hypot(offset[0], offset[1] ) # Calculates hypotenuse!
print(dist)


# Recognize tag1
# Image coords pixels
u = 30
v = 40

tag1Center = np.array([[u], 
                      [v]])


# Image coords pixels
u = 120
v = 40
tag2Center = np.array([[u], 
                      [v]])

tagDistance = tag2Center - tag1Center
