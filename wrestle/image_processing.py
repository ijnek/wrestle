#!/usr/bin/env python3
# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This module provides functions to work with images coming from the NAO's cameras.
"""

import numpy as np
import cv2
from vision_msgs.msg import BoundingBox2D

class ImageProcessing():
    @staticmethod
    def get_largest_contour(image):
        """Get the largest contour in an image."""
        contours, _ = cv2.findContours(
            image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if len(contours) == 0:
            return None
        return contours[0]

    @staticmethod
    def get_contour_centroid(contour):
        """Get the centroid of a contour."""
        M = cv2.moments(contour)
        if M['m00'] != 0:
            vertical_coordinate = int(M['m01'] / M['m00'])
            horizontal_coordinate = int(M['m10'] / M['m00'])
        else:
            # if the contour has an area of 0, the centroid cannot be computed this way
            # we use the mean of the contour points instead
            vertical_coordinate, horizontal_coordinate = np.mean(contour, axis=0)[0]
        return int(vertical_coordinate), int(horizontal_coordinate)

    @staticmethod
    def get_contour_min_v_coordinate(contour):
        """Get the centroid of a contour."""
        M = cv2.moments(contour)
        if M['m00'] != 0:
            vertical_coordinate = int(M['m01'] / M['m00'])
            horizontal_coordinate = int(M['m10'] / M['m00'])
        else:
            # if the contour has an area of 0, the centroid cannot be computed this way
            # we use the mean of the contour points instead
            vertical_coordinate, horizontal_coordinate = np.mean(contour, axis=0)[0]
        return int(vertical_coordinate), int(horizontal_coordinate)

    @staticmethod
    def get_bbox_2d(contour):
        """Get the bounding box of a contour."""
        x,y,w,h = cv2.boundingRect(contour)
        bb = BoundingBox2D()
        bb.center.position.x = float(x)
        bb.center.position.y = float(y)
        bb.size_x = float(w)
        bb.size_y = float(h)
        return bb

    @classmethod
    def locate_opponent(cls, img):
        """Image processing demonstration to locate the opponent robot in an image."""
        # we suppose the robot to be located at a concentration of multiple color changes (big Laplacian values)
        laplacian = cv2.Laplacian(img, cv2.CV_8U, ksize=3)
        # those spikes are then smoothed out using a Gaussian blur to get blurry blobs
        blur = cv2.GaussianBlur(laplacian, (0, 0), 2)
        # we apply a threshold to get a binary image of potential robot locations
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
        # the binary image is then dilated to merge small groups of blobs together
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)))
        # the robot is assumed to be the largest contour
        largest_contour = cls.get_largest_contour(closing)

        if largest_contour is not None:
            cv2.drawContours(img, [largest_contour], 0, (0,255,0), 3)
            cv2.imshow("img", img)
            cv2.waitKey(1)
            return cls.get_bbox_2d(largest_contour)
        else:
            return None

if __name__ == '__main__':
    img = cv2.imread('bla.png')
    # print(img)
    (contour, v_coord, h_coord) = ImageProcessing.locate_opponent(img)
    print(ImageProcessing.get_bbox_2d(contour))
    print(contour[0])
    cv2.drawContours(img, [contour], 0, (0,255,0), 3)
    cv2.imshow("img", img)
    cv2.waitKey(0)
