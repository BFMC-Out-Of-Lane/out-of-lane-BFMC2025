import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from src.hardware.camera.lane_detection.lane_detection_interface import \
    LaneDetectionInterface
from src.hardware.camera.lane_detection.line import Line
from scipy.signal import find_peaks

# from lane_detection_interface import LaneDetectionInterface
# from line import Line

MIN_POLY_PIXEL = 4000
NUM_WINDOWS = 12


def calculate_derivative(a, b, x):
    return 2 * a * x + b


class LaneDetector(LaneDetectionInterface):
    def __init__(self, min_offset, distance_to_aim, is_simu=False):
        self.min_offset = min_offset
        self.distance_to_aim = distance_to_aim
        self.is_simu = is_simu
        self.max_windows_overlapping = 8

        self.left_line = Line()
        self.right_line = Line()

    def process_image(self, cv_image):
        return self.process_frame(cv_image)

    def _threshold_image(self, image):
        # Apply Gaussian blur
        blurred_frame = cv2.GaussianBlur(image, (5, 5), 0)
        # cv2.imshow('Blurred Frame', blurred_frame)

        # Convert to HLS color space
        hls = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]

        # Threshold the L channel to get white colors
        _, white_mask = cv2.threshold(l_channel, 200, 255, cv2.THRESH_BINARY)

        # Edge detection using Sobel operator
        gray = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(sobelx)
        abs_sobelx = np.uint8(255 * abs_sobelx / (np.max(abs_sobelx if np.max(abs_sobelx) > 0 else 0.1)))

        # Threshold the Sobel image
        _, sxbinary = cv2.threshold(abs_sobelx, 100, 150, cv2.THRESH_BINARY)

        # Combine color and edge masks
        combined_binary = cv2.bitwise_or(white_mask, sxbinary)

        return combined_binary

    def _histogram(self, binary_warped):
        # histogram SHOULD be of the entire binary image because masking is done in the threshold_image function
        # however, a few rows should be removed at least in the simulator to avoid the hood of the car
        bottom_limit = int(binary_warped.shape[0]*0.97)
        histogram = np.sum(binary_warped[:bottom_limit], axis=0)
        self.histogram_data = histogram
        return histogram, None

    def _render_histogram(self, histogram):
        fig = plt.figure(figsize=(8, 4))
        plt.plot(histogram)
        plt.title('Histogram')
        plt.xlabel('Pixel Position')
        plt.ylabel('Summed Pixel Values')
        plt.tight_layout()

        # Render the plot to an image buffer
        fig.canvas.draw()
        # Convert to a numpy array
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)  # Close the figure to free memory

        # Convert to BGR format for OpenCV
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        # cv2.imshow('Histogram', img)
        return img

    def _sliding_window(self, binary_warped, histogram):
        self.sliding_windows_image = np.dstack((binary_warped, binary_warped, binary_warped))
        midpoint = int(histogram.shape[0] // 2)

        left_peaks = find_peaks(histogram[:midpoint], height=np.average(histogram[:midpoint]))[0]
        right_peaks = find_peaks(histogram[midpoint:], height=np.average(histogram[midpoint:]))[0]
        leftx_base = left_peaks[-1] if len(left_peaks) > 0 else midpoint - self.min_offset
        rightx_base = right_peaks[0] + midpoint if len(right_peaks) > 0 else midpoint + self.min_offset

        nwindows = NUM_WINDOWS
        window_height = int(binary_warped.shape[0] // nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        margin = 50
        minpix = 5  # Minimum number of pixels found to recenter window

        left_line_inds = []
        right_line_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height

            win_xleft_low = max(leftx_current - margin, 0)
            win_xleft_high = min(leftx_current + margin, binary_warped.shape[1])

            win_xright_low = max(rightx_current - margin, 0)
            win_xright_high = min(rightx_current + margin, binary_warped.shape[1])

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_line_inds.append(good_left_inds)
            right_line_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
                cv2.rectangle(self.sliding_windows_image, (win_xleft_low, win_y_low),
                              (win_xleft_high, win_y_high), (0, 255, 60), 2)
                
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                cv2.rectangle(self.sliding_windows_image, (win_xright_low, win_y_low),
                              (win_xright_high, win_y_high), (0, 255, 60), 2)

            if window == 0:
                self.bottom_left_window = (win_xleft_low + win_xleft_high) / 2
                self.bottom_right_window = (win_xright_low + win_xright_high) / 2

        left_line_inds = np.concatenate(left_line_inds)
        right_line_inds = np.concatenate(right_line_inds)

        leftx = nonzerox[left_line_inds]
        lefty = nonzeroy[left_line_inds]
        rightx = nonzerox[right_line_inds]
        righty = nonzeroy[right_line_inds]

        return leftx, lefty, rightx, righty

    def _fit_polynomial(self):
        hist, img = self._histogram(self.binary_warped)
        leftx, lefty, rightx, righty = self._sliding_window(self.binary_warped, hist)
        left_fit = None
        if len(leftx) > MIN_POLY_PIXEL:
            left_fit = np.polyfit(lefty, leftx, 2)

        right_fit = None
        if len(rightx) > MIN_POLY_PIXEL:
            right_fit = np.polyfit(righty, rightx, 2)
            
        return left_fit, right_fit, leftx, lefty, rightx, righty, img

    def _check_overlap(self, leftPoly, rightPoly):
        """"
        Checks if at least n windows overlap between the left and right lines (n = max_sliding_windows_overlapping)
        If they do, checks whether last window poinmt is on the left or right side of the image and deletes corresponding line
        """
        if leftPoly is None or rightPoly is None:
            return
        leftA, leftB, leftC = leftPoly
        rightA, rightB, rightC = rightPoly
        height = self.binary_warped.shape[0]
        step = height // NUM_WINDOWS

        num_sups = 0
        points = []
        for i in range(NUM_WINDOWS):
            y = i * step
            x_left = leftA * y ** 2 + leftB * y + leftC
            x_right = rightA * y ** 2 + rightB * y + rightC
            # Draw on the warped (sliding window) image
            cv2.circle(self.sliding_windows_image, (int(x_left), int(y)), 5, (255, 255, 0), -1)
            cv2.circle(self.sliding_windows_image, (int(x_right), int(y)), 5, (0, 255, 255), -1)
            diff = abs(x_right - x_left)
            superposition = diff < 100
            if superposition:
                num_sups += 1

            points.append((x_left, x_right))
            # print(
            # f"\033[34m A_left:\033[0m {leftA:2.5f}  "
            # f"\033[34m x_left:\033[0m {x_left:7.2f}  "
            # f"\033[31m A_right:\033[0m {rightA:2.5f}  "
            # f"\033[31m x_right:\033[0m {x_right:7.2f}  "
            # f"\033[33m diff:\033[0m {diff:7.2f}  "
            # f"\033[33m xL :\033[0m {x_left:7.2f}  "
            # f"\033[33m yL :\033[0m {y:7.2f}  "
            # f"\033[35m super:\033[0m {superposition}")
        if num_sups > self.max_windows_overlapping:
            x_left, x_right = points[-1]
            x_center = (x_left + x_right) // 2
            stay_with = "left" if x_center < self.binary_warped.shape[1] // 2 else "right"
            if stay_with == "left":
                self.right_line.best_fit = None
            else:
                self.left_line.best_fit = None

    def _draw_lane(self, original_image, binary_warped):
        left_fit = self.left_line.best_fit
        right_fit = self.right_line.best_fit

        # Generate y-values for plottings
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        result = original_image.copy()

        left_fitx = None
        right_fitx = None
        if left_fit is None and right_fit is None:
            return original_image

        if left_fit is not None:
            # Calculate x-values for each y-value using the polynomial coefficients
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            # Create points for the line
            left_line_pts = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            # Warp the lane line back onto the original image
            left_line_pts = cv2.perspectiveTransform(left_line_pts, self.Minv)
            # c Drawing
            cv2.polylines(result, [np.int32(left_line_pts)], isClosed=False, color=(255, 0, 0), thickness=10)

        if right_fit is not None:
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
            right_line_pts = np.array([np.transpose(np.vstack([right_fitx, ploty]))])
            right_line_pts = cv2.perspectiveTransform(right_line_pts, self.Minv)
            cv2.polylines(result, [np.int32(right_line_pts)], isClosed=False, color=(0, 0, 255), thickness=10)

        return result

    def _get_perspective_transform(self, image):
        height, width = image.shape[:2]
        if self.is_simu:
            self.top_left = [170, 300]
            self.top_right = [450, 300]
            self.bottom_right = [width * 1, height*0.98]
            self.bottom_left = [width * 0, height*0.98]
        else:
            self.top_left = [width * 0.3, height * 0.8]
            self.top_right = [width * 0.7, height * 0.8]
            self.bottom_right = [width * 0.8, height * 0.98]
            self.bottom_left = [width * 0.2, height * 0.98]
            
        src = np.float32([
            self.top_left,
            self.top_right,
            self.bottom_right,
            self.bottom_left
        ])
        dst = np.float32([
            [width * 0.2, 0],
            [width * 0.8, 0],
            [width * 0.8, height],
            [width * 0.2, height]
        ])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)  # For inverse transform

    def _warp_image(self, image, M):
        height, width = image.shape[:2]
        warped = cv2.warpPerspective(image, M, (width, height))
        return warped

    def _calculate_tracking_params(self):
        left_fit = self.left_line.best_fit
        right_fit = self.right_line.best_fit
        self.cross_tracking_error = self.deviation = None

        if left_fit is not None and right_fit is not None:
            self.cross_tracking_error = (self.bottom_left_window + self.bottom_right_window) // 2
            self.deviation = calculate_derivative(
                (left_fit[0] + right_fit[0]) / 2, (left_fit[1] + right_fit[1]) / 2, 242)
        elif left_fit is not None:
            self.cross_tracking_error = self.bottom_left_window + self.min_offset
            self.deviation = calculate_derivative(left_fit[0], left_fit[1], 242)
        elif right_fit is not None:
            self.cross_tracking_error = self.bottom_right_window - self.min_offset
            self.deviation = calculate_derivative(right_fit[0], right_fit[1], 242)

        if self.cross_tracking_error and self.deviation:
            self.cross_tracking_error = self.cross_tracking_error - self.middle
            self.deviation = - math.atan(self.deviation) * 180 / math.pi

    def process_frame(self, frame):
        self._get_perspective_transform(frame)
        warped = self._warp_image(frame, self.M)
        self.binary_warped = self._threshold_image(warped)
        # binary_unwarped = self.warp_image(self.binary_warped, self.Minv)
        # cv2.imshow('Binary Unwarped', binary_unwarped)
        # cv2.imshow('Binary Warped', binary_warped)
        
        left_fit, right_fit, leftx, lefty, rightx, righty, histImg = self._fit_polynomial()
        
        # cv2.imshow('Sliding Window', self.sliding_windows_image)

        self.left_line.add_fit(left_fit)
        self.right_line.add_fit(right_fit)
        self._check_overlap(self.left_line.best_fit, self.right_line.best_fit)

        self.middle = frame.shape[1] // 2
        self._calculate_tracking_params()

        result = self._draw_lane(frame, self.binary_warped)
        
        # hist_image = self.render_histogram(self.histogram_data)
        return self.sliding_windows_image, self.cross_tracking_error, self.deviation
