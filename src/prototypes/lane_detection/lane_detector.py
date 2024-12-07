import cv2
import numpy as np
from line_detector import LineDetector

FILENAME = 'test2.mp4'

PIXEL_PRECISSION = 2
RADIAN_PRECISSION = np.pi / 180
THRESHOLD = 100
MIN_LINE_LENGTH = 50
MAX_LINE_GAP = 50
END_OF_LINES = 3 / 5


class LaneDetector:

    def __init__(self, roi):
        self.line_detector = LineDetector(
            PIXEL_PRECISSION, RADIAN_PRECISSION, THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP, END_OF_LINES, roi)

    def detect_lane(self, image):
        return self.line_detector.detect_lines(image)

    def display_lines_on_image(self, lines):
        return self.line_detector.display_average_lines_on_image(lines)


if __name__ == '__main__':

    cap = cv2.VideoCapture(FILENAME)

    roi = [
        (550, 330), 
        (190, 720),
        (1100, 720), 
        (600, 330), 
        ]
    lane_detector = LaneDetector(roi)

    while cap.isOpened():
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        _, frame = cap.read()
        image_w_lines = lane_detector.display_lines_on_image(frame)
        cv2.imshow('image', image_w_lines)
        # cv2.imshow('warped', lane_detector.line_detector.warped_image.T)

    # image = cv2.imread('test1.jpg')
    # lane_detector = LaneDetector([
    #     (274, 184),  # Top-left corner
    #     (0, 337),  # Bottom-left corner
    #     (575, 337),  # Bottom-right corner
    #     (371, 184)  # Top-right corner
    # ])
    # image_w_lines = lane_detector.display_lines_on_image(image)
    # cv2.imshow('image', image_w_lines)
    # cv2.imshow('warped', lane_detector.line_detector.warped_image)
    # cv2.waitKey(0)
