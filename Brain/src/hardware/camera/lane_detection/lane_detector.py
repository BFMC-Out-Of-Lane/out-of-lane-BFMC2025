import math

import cv2
import matplotlib.pyplot as plt
import numpy as np

from scipy.signal import find_peaks
#from src.hardware.camera.lane_detection.line import Line

# from lane_detection_interface import LaneDetectionInterface
from line import Line

HIGH_POINT = 10
MIDDLE_POINT = 50
LOW_POINT = 100
SOME_DISTANCE_BETWEEN_POINTS = 100

def calculate_derivative(a, b, x):
    return 2 * a * x + b


class LaneDetector:
    def __init__(self, min_offset, perc_of_sight_len):
        self.min_offset = min_offset
        self.perc_of_sight_len = perc_of_sight_len
        self.traffic_sign = None

        self.MAX_WINDOWS_OVERLAPPING = 8
        self.NUM_OF_WINDOWS = 12
        self.MIN_POLY_PIXEL = 1500

        self.left_line = Line()
        self.right_line = Line()

    def _threshold_image(self, image):
        # Apply Gaussian blur
        blurred_frame = cv2.GaussianBlur(image, (5, 5), 0)

        # Convert to HLS color space
        hls = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]

        # Threshold the L channel to get white colors
        _, white_mask = cv2.threshold(l_channel, 150, 255, cv2.THRESH_BINARY)

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
        #histogram = histogram / histogram.sum()
        lower_threshold = 0.001
        upper_threshold = 0.003
        flat_area = np.where((histogram > lower_threshold) & (histogram < upper_threshold))[0]
        for intensity in flat_area:
            histogram[intensity] = 0
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

        return img

    def _sliding_window(self, binary_warped, histogram):
        self.sliding_windows_image = np.dstack((binary_warped, binary_warped, binary_warped))
        midpoint = int(histogram.shape[0] // 2)

        left_peaks = find_peaks(histogram[:midpoint], height=0)[0]
        right_peaks = find_peaks(histogram[midpoint:], height=0)[0]
        if self.traffic_sign is not None and "crosswalk-road" in self.traffic_sign and self.traffic_sign["crosswalk-road"] > 50:
            leftx_base = left_peaks[0] if len(left_peaks) > 0 else midpoint - self.min_offset
            rightx_base = right_peaks[-1] + midpoint if len(right_peaks) > 0 else midpoint + self.min_offset
        else:
            leftx_base = left_peaks[-1] if len(left_peaks) > 0 else midpoint - self.min_offset
            rightx_base = right_peaks[0] + midpoint if len(right_peaks) > 0 else midpoint + self.min_offset

        nwindows = self.NUM_OF_WINDOWS
        window_height = int(binary_warped.shape[0] // nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        margin = 50
        minpix = 1  # Minimum number of pixels found to recenter window

        left_line_inds = []
        right_line_inds = []
        # ---------- DANGER
        win_y_low = binary_warped.shape[0] - window_height
        win_y_high = binary_warped.shape[0]

        win_xleft_low = max(leftx_current - margin, 0)
        win_xleft_high = min(leftx_current + margin, binary_warped.shape[1])

        win_xright_low = max(rightx_current - margin, 0)
        win_xright_high = min(rightx_current + margin, binary_warped.shape[1])

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
            cv2.rectangle(self.sliding_windows_image, (win_xleft_low, win_y_low),
                          (win_xleft_high, win_y_high), (60, 255, 250), 2)

        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))
            cv2.rectangle(self.sliding_windows_image, (win_xright_low, win_y_low),
                          (win_xright_high, win_y_high), (60, 255, 250), 2)
        # ---------- DANGER
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
        if len(leftx) > self.MIN_POLY_PIXEL:
            left_fit = np.polyfit(lefty, leftx, 2)

        right_fit = None
        if len(rightx) > self.MIN_POLY_PIXEL:
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
        step = height // self.NUM_OF_WINDOWS

        num_sups = 0
        points = []
        for i in range(self.NUM_OF_WINDOWS):
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
        if num_sups > self.MAX_WINDOWS_OVERLAPPING:
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
            # Draw circles along the polynomial
            self._draw_circles_along_polynomial(result, left_fit, (255, 255, 0), [HIGH_POINT, MIDDLE_POINT, LOW_POINT])
            # Drawing the line
            cv2.polylines(result, [np.int32(left_line_pts)], isClosed=False, color=(255, 0, 0), thickness=10)

        if right_fit is not None:
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
            right_line_pts = np.array([np.transpose(np.vstack([right_fitx, ploty]))])
            right_line_pts = cv2.perspectiveTransform(right_line_pts, self.Minv)
            # Draw circles along the polynomial
            self._draw_circles_along_polynomial(result, right_fit, (0, 255, 255), [HIGH_POINT, MIDDLE_POINT, LOW_POINT])
            cv2.polylines(result, [np.int32(right_line_pts)], isClosed=False, color=(0, 0, 255), thickness=10)

        return result

    def _draw_circles_along_polynomial(self, result, fit, color, positions=[10, 50, 100]):
        """
        Draw circles along a polynomial curve at specified positions.
        
        Args:
            result: The image to draw on
            fit: The polynomial coefficients (a, b, c)
            color: BGR color tuple for the circles
            positions: List of percentages from top of image (0-100)
        """
        for pos in positions:
            y = int(result.shape[0] * pos / 100)  # Convert percentage to y coordinate
            x = int(fit[0] * y ** 2 + fit[1] * y + fit[2])
            point = np.array([[[x, y]]], dtype=np.float32)
            warped_point = cv2.perspectiveTransform(point, self.Minv)
            cv2.circle(result, (int(warped_point[0][0][0]), int(warped_point[0][0][1])), 10, color, -1)

    def _get_perspective_transform(self, image):
        height, width = image.shape[:2]

        self.top_left = [width * 0.3, height * (1 - self.perc_of_sight_len)]
        self.top_right = [width * 0.7, height * (1 - self.perc_of_sight_len)]
        self.bottom_right = [width * 0.9, height * 0.95]
        self.bottom_left = [width * 0.1, height * 0.95]

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

    def process_frame(self, frame, traffic_sign):
        if traffic_sign is not None:
            self.traffic_sign = traffic_sign["objectDict"]
        self._get_perspective_transform(frame)
        warped = self._warp_image(frame, self.M)
        self.binary_warped = self._threshold_image(warped)
        # binary_unwarped = self.warp_image(self.binary_warped, self.Minv)

        left_fit, right_fit, leftx, lefty, rightx, righty, histImg = self._fit_polynomial()

        self.left_line.add_fit(left_fit)
        self.right_line.add_fit(right_fit)
        self._check_overlap(self.left_line.best_fit, self.right_line.best_fit)
        # cosito para paralelismo
        self.are_lanes_parallel(self.left_line.best_fit, self.right_line.best_fit)
        # --------------------------------------------------------------
        self.middle = frame.shape[1] // 2
        self._calculate_tracking_params()

        result = self._draw_lane(frame, self.binary_warped)

        # hist_image = self._render_histogram(self.histogram_data)

        return result, self.cross_tracking_error, self.deviation, self.binary_warped
    
    def are_lanes_parallel(self, left_fit, right_fit):
        if left_fit is None or right_fit is None:
            print("No fit")
            return 
        
        # DIFFERENCE BETWEEN COEFFICIENTS --------------------------------
        diff = left_fit - right_fit
        #print(f"Quadratic term (a) difference: {diff[0]:.4f}, Linear term (b) difference: {diff[1]:.4f}, Constant term (c) difference: {diff[2]:.4f}")
        # --------------------------------------------------------------
        # DIFFERENCE BETWEEN HIGH POINTS --------------------------------
        left_high = left_fit[0] * HIGH_POINT ** 2 + left_fit[1] * HIGH_POINT + left_fit[2]
        right_high = right_fit[0] * HIGH_POINT ** 2 + right_fit[1] * HIGH_POINT + right_fit[2]
        #print("Difference between high points: ", right_high - left_high)
        # --------------------------------------------------------------
        ## DIFFERENCE BETWEEN DERIVATIVES -------------------------------- Es el que mas me gusta, aprox cuando la dif es -1, es cuando se rompe el detector
        left_derivative = calculate_derivative(left_fit[0], left_fit[1], MIDDLE_POINT) 
        right_derivative = calculate_derivative(right_fit[0], right_fit[1], MIDDLE_POINT)
        print("Difference between derivatives: ", right_derivative - left_derivative)
        # --------------------------------------------------------------qq
      
            

if __name__ == "__main__":
    # Initialize the lane detector
    detector = LaneDetector(min_offset=60, perc_of_sight_len=0.3)
    
    # Open the video file
    cap = cv2.VideoCapture('Brain/src/hardware/camera/lane_detection/video.mp4')
    
    # Check if video opened successfully
    if not cap.isOpened():
        print("Error: Could not open video file")
        exit()
    
    while True:
        # Read a frame
        ret, frame = cap.read()
        
        # Break the loop if we've reached the end of the video
        if not ret:
            break
            
        # Process the frame
        result, cross_tracking_error, deviation, binary_warped = detector.process_frame(frame, None)
        
        # Display the results
        cv2.imshow('Processed Frame', result)
        cv2.imshow('Binary Warped', binary_warped)
        
        # Print tracking parameters
        #print("--------------------------------")
        #print(f"Cross Tracking Error: {cross_tracking_error}")
        #print(f"Deviation: {deviation}")
        #print("--------------------------------")
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    
