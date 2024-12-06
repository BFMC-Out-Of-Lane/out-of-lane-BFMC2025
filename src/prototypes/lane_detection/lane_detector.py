import cv2
import numpy as np
from line_detector import LineDetector

FILENAME = 'test1.jpg'

PIXEL_PRECISSION = 2
RADIAN_PRECISSION = np.pi / 180
THRESHOLD = 180
MIN_LINE_LENGTH = 40
MAX_LINE_GAP = 5


class LaneDetector:

    def __init__(self):
        self.line_detector = LineDetector(
            PIXEL_PRECISSION, RADIAN_PRECISSION, THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP)

    def detect_lane(self, image):
        lines = self.line_detector.detect_lines(image)
        # averaged_lines = self._average_slope_intercept(image, lines)
        return lines

    def display_lines_on_image(self, lines):
        return self.line_detector.display_average_lines_on_image(lines)


if __name__ == '__main__':
    image = cv2.imread(FILENAME)
    lane_detector = LaneDetector()
    lines = lane_detector.detect_lane(image)

    cv2.imshow('frame', lane_detector.display_lines_on_image(lines))
    cv2.waitKey(0)
    cv2.destroyWindow('frame')
