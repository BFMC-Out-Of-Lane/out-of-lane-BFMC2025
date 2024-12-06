import cv2
import numpy as np


class LineDetector:

    def __init__(self, pixel_precission, radian_precission, threshold, min_line_length, max_line_length):
        self.pixel_precission = pixel_precission
        self.radian_precission = radian_precission
        self.threshold = threshold
        self.min_line_length = min_line_length
        self.max_line_length = max_line_length

    def _detect_edges(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)

        return canny

    def _apply_roi(self, image):
        height = self.image.shape[0]
        triangle = np.array([(20, height), (564, height), (330, 160)])
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, [triangle], 255)
        masked_image = cv2.bitwise_and(image, mask)

        return masked_image

    def _to_coordinates(self, line_parameters):
        slope, intercept = line_parameters
        y1 = self.image.shape[0]
        y2 = int(y1 * 3 / 5)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return np.array([x1, y1, x2, y2])

    def _average_slope_intercept(self, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line = self._to_coordinates(left_fit_average)
        right_line = self._to_coordinates(right_fit_average)

        return np.array([left_line, right_line])

    def detect_lines(self, image):
        self.image = np.copy(image)

        edged_image = self._detect_edges(self.image)
        cropped_image = self._apply_roi(edged_image)
        lines = cv2.HoughLinesP(cropped_image, self.pixel_precission, self.radian_precission, self.threshold, np.array(
            []), minLineLength=self.min_line_length, maxLineGap=self.max_line_length)

        return lines

    def display_lines(self, lines):
        line_image = np.zeros_like(self.image)
        if lines.all():
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)

        return line_image

    def display_lines_on_image(self, lines):
        return cv2.addWeighted(self.image, 0.8, self.display_lines(lines), 1, 1)

    def display_average_lines(self, lines):
        lines = self._average_slope_intercept(lines)
        line_image = np.zeros_like(self.image)
        if lines.all():
            for line in lines:
                x1, y1, x2, y2 = line
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)

        return line_image

    def display_average_lines_on_image(self, lines):
        return cv2.addWeighted(self.image, 0.8, self.display_average_lines(lines), 1, 1)
