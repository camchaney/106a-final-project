#!/usr/bin/env python3

import cv2
import numpy as np


class FeatureExtractor(object):
    
    def __init__(
            self,
            min_contour_len=100,
            max_image_height=500,
            threshold_sample_size=201
        ):
        """
        min_contour_len: The minimum accepted length of a contour. We don't want a ton of noisy little lines everywhere
        max_image_height: Normalizing constant for the size of the image
        threshold_sample_size: The sample area that adaptive thresholding looks at
        """
        self.min_contour_len = min_contour_len
        self.max_image_height = max_image_height
        self.threshold_sample_size = threshold_sample_size

    def extract_contour(self, img):
        """
        extracts the contours found in 'img'
        """
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
        contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours, hierarchy

    def filter_contours_by_len(self, contours):
        """
        Removes all contours shorter than 'self.min_contour_len'
        """
        new_contours = []
        for contour in contours:
            if cv2.arcLength(contour, True) > self.min_contour_len:
                new_contours.append(contour)
        return tuple(new_contours)

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
