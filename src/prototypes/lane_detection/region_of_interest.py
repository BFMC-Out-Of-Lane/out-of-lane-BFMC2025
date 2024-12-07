import cv2
import numpy as np


class RegionOfInterest:
    # TODO: This class shoud calculate the mask needed for "_apply_roi" method in LineDetector class and also the matrix needed for "_warp_image" method in LineDetector class

    def __init__(self, vertices):
        self.vertices = np.array(vertices)

    def mask(self, image):
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, [self.vertices], 255)
        return cv2.bitwise_and(image, mask)
