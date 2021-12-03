import cv2
import numpy as np
import shapely

## Largely pulled from this article by Om Singh
## https://blog.electroica.com/eye-dropper-color-picker-tool-in-python-using-opencv/

class boxPicker:

    def __init__(self):
        pass

    def pickBox(self, img, boxList, truthList):
        self.img = img
        self.imgName = "Box Picker"
        self.boxList = boxList
        self.truthList = truthList
		#image window for sample image
		cv2.namedWindow(self.imgName)

		#mouse call back function declaration
		cv2.setMouseCallback(self.imgName, self.selectBox)

		print("Left click to choose a color, spacebar to save and exit")

		#while loop to live update
		while (1):
			cv2.imshow(self.imgName, self.img)

			k = cv2.waitKey(1) & 0xFF
			if k == 27: # ESC
				self.destroy_color_picker()
				return None
			elif k == 32: # space
				self.destroy_color_picker()

	def destroy_color_picker(self):
		cv2.destroyWindow("Color Picker")

    def insideBox(self, x, y, box):
        myPolygon = shapely.Polygon(box)
        return myPolygon.contains((x, y))

	# Mouse Callback function - this is triggered every time the mouse moves
	def selectBox(self, event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            for box in self.boxList:
                if self.insideBox(x, y, box):
                    self.whenBoxSelected()