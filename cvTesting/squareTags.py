# import the necessary packages
import argparse
import imutils
import numpy as np
import cv2
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
    help="path to the input image")
args = vars(ap.parse_args())


# class ShapeDetector:
#     def __init__(self):
# 	    pass

#     def detect(self, c):
# 		# initialize the shape name and approximate the contour
#         shape = "unidentified"
#         peri = cv2.arcLength(c, True)
#         approx = cv2.approxPolyDP(c, 0.04 * peri, True)
#         # if the shape is a triangle, it will have 3 vertices
# 	    #if len(approx) == 3:
# 		#    shape = "triangle"
# 		# if the shape has 4 vertices, it is either a square or
# 		# a rectangle
#         if len(approx) == 4:
# 			# compute the bounding box of the contour and use the
# 			# bounding box to compute the aspect ratio
#             (x, y, w, h) = cv2.boundingRect(approx)
#             print(x, y, w, h)
#             print(f"Ratio of w/h: {w/h}")
#             ar = w / float(h)
# 			# a square will have an aspect ratio that is approximately
# 			# equal to one, otherwise, the shape is a rectangle
#             shape = "square" #if ar >= 0.95 and ar <= 1.05 else "rectangle"
#         # if the shape is a pentagon, it will have 5 vertices
#         else:
#             shape = len(approx)+"-agon"
# 		# otherwise, we assume the shape is a circle
#         return shape


# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread(args["image"])
if not image.any():
    print("Couldn't load image")

resized = image #imutils.resize(image, width=600)
ratio = image.shape[0] / float(resized.shape[0])
print(ratio)



hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_red = np.array([30,150,50])
upper_red = np.array([255,255,180])

lower_green = np.array([36,0,0])
upper_green = np.array([86,255,255])

lower_yellow = np.array([30,0,0])
upper_yellow = np.array([36,255,255])

greenMask = cv2.inRange(hsv, lower_green, upper_green)
greenRes = cv2.bitwise_and(image,image, mask= greenMask)

yellowMask = cv2.inRange(hsv, lower_yellow, upper_yellow)
yellowRes = cv2.bitwise_and(image,image, mask= yellowMask)

cv2.imshow("HSV Green", greenMask)  # Tag 1
cv2.imshow("HSV Yellow", yellowMask) # Red Tag 2


cnts = cv2.findContours(yellowMask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

# THRESHOLD FOR DETECTING A SQUARE
# Not looking at Color Now
widthHeightLow = .9
widthHeightHigh = 1.1

originTag = None
rightTag = None

# loop over the contours
for c in cnts:
	# compute the center of the contour, then detect the name of the
	# shape using only the contour
    rot_rect = cv2.minAreaRect(c)

    # This isn't Valid
    if rot_rect[1][0] == 0 or rot_rect[1][0] == 0:
        continue

    widthHeightRatio = rot_rect[1][0]/rot_rect[1][1] 
    if widthHeightRatio < widthHeightHigh and widthHeightRatio > widthHeightLow:
        #Add a check to see if it about the right size!!
        aTag = True
    else:
        aTag = False

    if aTag:
        pass

    #rotation = rot_rect[2]
    print(f"Center(x,y): {rot_rect[0]}, Width,Height: {rot_rect[1]}")
    # if rot_rect[1][0] <= rot_rect[1][1]:
    #     # This means the block is in a certain orientation
    #     print("Apply offset ")
    #     rotation += 90


    box = cv2.boxPoints(rot_rect) * ratio
    box = np.int0(box)
    print(box)
    # draw rotated rectangle on copy of img
    rot_bbox = image.copy()
    cv2.drawContours(rot_bbox,[box],0,(0,0,255),2)
    cv2.imshow("Image", rot_bbox)

    # Calculate Center



    # M = cv2.moments(c)
    # cX = int((M["m10"] / M["m00"]) * ratio)
    # cY = int((M["m01"] / M["m00"]) * ratio)
    # shape = sd.detect(c)
	# # multiply the contour (x, y)-coordinates by the resize ratio,
	# # then draw the contours and the name of the shape on the image
    # c = c.astype("float")
    # c *= ratio
    # c = c.astype("int")
    # # cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
    # # cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
	# # 	0.5, (255, 255, 255), 2)
	# # # show the output image
    # cv2.imshow("Image", image)
    cv2.waitKey(0)