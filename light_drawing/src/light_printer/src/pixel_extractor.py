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
        self.num_leds = 26
        self.pixel_spacing = 6.48       # pixel spacing (mm)

    def create_contour(self, img):
        """
        creates contours for given image
        """
        while 
        grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayscale = cv2.blur(grayscale, (5, 5))
        thresh_img = cv2.adaptiveThreshold(
            grayscale,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            self.threshold_sample_size,
            0,
        )
        return contours


    def resize_img(self, img):
        """
        Normalizes the size of the image
        """
        rows = img.shape[0]
        columns = img.shape[1]
        if columns < self.max_image_height:
            return img
        else:
            scale = self.max_image_height / columns
            return cv2.resize(img, (int(columns * scale), int(rows * scale)))

    def get_colors(self, img):
        """
        
        img: img that has been resized down to number of LEDs
        """
        width = img.shape[1]
        height = img.shape[2]
        height_left = height
        colors = []
        while height_left < 0:
            if 
            colors.append(height)
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
