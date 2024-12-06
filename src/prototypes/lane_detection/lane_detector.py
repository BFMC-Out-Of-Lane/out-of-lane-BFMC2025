import cv2
import numpy as np
from line_detector import LineDetector

FILENAME = 'test2.mp4'

PIXEL_PRECISSION = 2
RADIAN_PRECISSION = np.pi / 180
THRESHOLD = 100
MIN_LINE_LENGTH = 50
MAX_LINE_GAP = 50


class LaneDetector:

    def __init__(self, roi):
        self.line_detector = LineDetector(
            PIXEL_PRECISSION, RADIAN_PRECISSION, THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP, roi)

    def detect_lane(self, image):
        return self.line_detector.detect_lines(image)

    def display_lines_on_image(self, lines):
        return self.line_detector.display_average_lines_on_image(lines)


if __name__ == '__main__':

    cap = cv2.VideoCapture(FILENAME)

    roi = [(170, 720), (1100, 720), (560, 285)]
    lane_detector = LaneDetector(roi)

    while cap.isOpened():
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        _, frame = cap.read()
        lines = lane_detector.detect_lane(frame)
        cv2.imshow('frame', lane_detector.display_lines_on_image(lines))
