import cv2
import numpy as np

## Largely pulled from this article by Om Singh
## https://blog.electroica.com/eye-dropper-color-picker-tool-in-python-using-opencv/

class ColorPicker:

	def __init__(self, img):
		self.img = img
		self.color_explore = np.zeros((150,150,3), np.uint8)
		self.color_selected = np.zeros((150,150,3), np.uint8)

	def colorPicker(self):

		#live update color with cursor
		cv2.namedWindow('color_explore')
		cv2.resizeWindow("color_explore", 50,50)

		#Show selected color when left mouse button pressed
		cv2.namedWindow('color_selected')
		cv2.resizeWindow("color_selected", 50,50)

		#image window for sample image
		cv2.namedWindow('Color Picker')

		#mouse call back function declaration
		cv2.setMouseCallback('Color Picker', self.show_color)

		print("Left click to choose a color, spacebar to save and exit")

		#while loop to live update
		while (1):
			cv2.imshow('Color Picker', self.img)
			cv2.imshow('color_explore', self.color_explore)
			cv2.imshow('color_selected', self.color_selected)

			k = cv2.waitKey(1) & 0xFF
			if k == 27: # ESC
				self.destroy_color_picker()
				return None
			elif k == 32: # space
				self.destroy_color_picker()
				color_selected_hsv = cv2.cvtColor(self.color_selected, cv2.COLOR_BGR2HSV)
				H, S, V = self.extract_color(color_selected_hsv)
				return [H, S, V]

	def destroy_color_picker(self):
		cv2.destroyWindow("Color Picker")
		cv2.destroyWindow("color_explore")
		cv2.destroyWindow("color_selected")


	def extract_color(self, color_grid):
		B = color_grid[10,10][0]
		G = color_grid[10,10][1]
		R = color_grid[10,10][2]
		return B, G, R

	#Mouse Callback function - this is triggered every time the mouse moves
	def show_color(self, event,x,y,flags,param):
		B = self.img[y,x][0]
		G = self.img[y,x][1]
		R = self.img[y,x][2]
		self.color_explore [:] = (B,G,R)

		if event == cv2.EVENT_LBUTTONDOWN:
			self.color_selected [:] = (B,G,R)


## Use case:
#
# # sample image path
# img_path="C:/Users/ieykamp/Downloads/WIN_20211122_13_44_37_Pro.png"

# #read sample image
# img = cv2.imread(img_path)
# cp = ColorPicker(img)
# print(cp.colorPicker())