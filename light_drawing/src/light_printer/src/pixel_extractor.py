#!/usr/bin/env python3

import cv2
import numpy as np


class PixelExtractor(object):
    
    def __init__(
            self,
            max_image_height=500,
            image_width=162
        ):
        """
        max_image_height: Normalizing constant for the size of the image
        threshold_sample_size: The sample area that adaptive thresholding looks at
        """
        self.max_image_height = max_image_height
        self.image_width = image_width
        self.num_pixels = 26
        self.pixel_spacing = 6.48       # pixel spacing (mm)


    def resize_img(self, img):
        """
        Normalizes the size of the image
        """
        x = img.shape[0]
        y = img.shape[1]
        if y < self.max_image_height:
            return img
        else:
            scale = self.max_image_height / y
            return cv2.resize(img, (int(y * scale), int(x * scale)))

    def invert_black_white(self, img):
        """
        Inverts the contour image so it is white contours on black
        """
        return abs(img - 255)

    def create_empty_img(self, x, y):
        """
        Creates an empty image array of size 'x' by 'y'
        """
        return np.ones((x, y)) * 255

    def draw_image(self, img, name):
        """
        Draws the input image 'img' with figure title 'name'
        """
        cv2.imshow(name, img)
        cv2.waitKey(0)
