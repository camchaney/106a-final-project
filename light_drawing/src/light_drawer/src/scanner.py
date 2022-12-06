from skimage.filters import threshold_local
from feature_extractor import FeatureExtractor
import numpy as np
import argparse
import cv2

class Scanner(object):
	def __init__(
			self,
			img_height=600
		):
		"""
		min_contour_len: The minimum accepted length of a contour. We don't want a ton of noisy little lines everywhere
		max_image_height: Normalizing constant for the size of the image
		threshold_sample_size: The sample area that adaptive thresholding looks at
		"""
		self.img_height = img_height

	def order_points(self, pts):
		# initialzie a list of coordinates that will be ordered
		# such that the first entry in the list is the top-left,
		# the second entry is the top-right, the third is the
		# bottom-right, and the fourth is the bottom-left
		rect = np.zeros((4, 2), dtype = "float32")
		# the top-left point will have the smallest sum, whereas
		# the bottom-right point will have the largest sum
		s = pts.sum(axis = 1)
		rect[0] = pts[np.argmin(s)]
		rect[2] = pts[np.argmax(s)]
		# now, compute the difference between the points, the
		# top-right point will have the smallest difference,
		# whereas the bottom-left will have the largest difference
		diff = np.diff(pts, axis = 1)
		rect[1] = pts[np.argmin(diff)]
		rect[3] = pts[np.argmax(diff)]
		# return the ordered coordinates
		return rect

	def four_point_transform(self, image, pts):
		# obtain a consistent order of the points and unpack them
		# individually
		rect = self.order_points(pts)
		(tl, tr, br, bl) = rect
		# compute the width of the new image, which will be the
		# maximum distance between bottom-right and bottom-left
		# x-coordiates or the top-right and top-left x-coordinates
		widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
		widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
		maxWidth = max(int(widthA), int(widthB))
		# compute the height of the new image, which will be the
		# maximum distance between the top-right and bottom-right
		# y-coordinates or the top-left and bottom-left y-coordinates
		heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
		heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
		maxHeight = max(int(heightA), int(heightB))
		# now that we have the dimensions of the new image, construct
		# the set of destination points to obtain a "birds eye view",
		# (i.e. top-down view) of the image, again specifying points
		# in the top-left, top-right, bottom-right, and bottom-left
		# order
		dst = np.array([
			[0, 0],
			[maxWidth - 1, 0],
			[maxWidth - 1, maxHeight - 1],
			[0, maxHeight - 1]], dtype = "float32")
		# compute the perspective transform matrix and then apply it
		M = cv2.getPerspectiveTransform(rect, dst)
		warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
		# return the warped image
		return warped

	def find_screen(self, image, cnts):
		cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
		screenCnt = None
		# loop over the contours
		for c in cnts:
			# approximate the contour
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.02 * peri, True)
			# if our approximated contour has four points, then we
			# can assume that we have found our screen
			if len(approx) == 4:
				screenCnt = approx
				break
		return screenCnt

	def transform_image(self, image, screenCnt):
		""" 
		Given the original image and the screen countor return the warped image
		"""
		ratio = image.shape[0] / self.img_height	
		warped = self.four_point_transform(image, screenCnt.reshape(4, 2) * ratio)
		return warped
	 