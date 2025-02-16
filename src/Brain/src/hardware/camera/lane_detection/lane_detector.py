import cv2
import matplotlib.pyplot as plt
import numpy as np
from src.hardware.camera.lane_detection.lane_detection_interface import \
    LaneDetectionInterface
from src.hardware.camera.lane_detection.line import Line

# from lane_detection_interface import LaneDetectionInterface
# from line import Line


class LaneDetector(LaneDetectionInterface):
    def __init__(self, min_offset, distance_to_aim, is_simu=False):
        self.min_offset = min_offset
        self.distance_to_aim = distance_to_aim
        self.is_simu = is_simu

        self.left_line = Line()
        self.right_line = Line()
        self.average_line = Line()

    def process_image(self, cv_image):
        return self.process_frame(cv_image)

    def threshold_image(self, image):
        # Apply Gaussian blur
        blurred_frame = cv2.GaussianBlur(image, (5, 5), 0)
        # cv2.imshow('Blurred Frame', blurred_frame)

        # Convert to HLS color space
        hls = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]

        # Threshold the L channel to get white colors
        _, white_mask = cv2.threshold(l_channel, 200, 255, cv2.THRESH_BINARY)

        # Threshold the S channel to get yellow colors
        s_channel = hls[:, :, 2]
        lower_yellow = np.array([15, 30, 115])
        upper_yellow = np.array([35, 204, 255])
        yellow_mask = cv2.inRange(hls, lower_yellow, upper_yellow)

        # Combine the masks
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # Edge detection using Sobel operator
        gray = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(sobelx)
        abs_sobelx = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))

        # Threshold the Sobel image
        _, sxbinary = cv2.threshold(abs_sobelx, 20, 100, cv2.THRESH_BINARY)

        # Combine color and edge masks
        combined_binary = cv2.bitwise_or(color_mask, sxbinary)

        return combined_binary

    def histogram(self, binary_warped):
        # histogram SHOULD be of the entire binary image because masking is done in the threshold_image function
        # however, a few rows should be removed at least in the simulator to avoid the hood of the car
        bottom_limit = int(binary_warped.shape[0]*0.97)
        # cv2.line(binary_warped, (0, bottom_limit), (binary_warped.shape[1], bottom_limit), (255, 0, 0), 2)
        histogram = np.sum(binary_warped[:bottom_limit], axis=0)
        # cv2.imshow('Histogram', self.render_histogram(histogram))
        return histogram, None

    def render_histogram(self, histogram):
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

    def sliding_window(self, binary_warped, histogram):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))
        midpoint = int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = int(binary_warped.shape[0] // nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base

        margin = 80
        minpix = 50  # Minimum number of pixels found to recenter window

        left_line_inds = []
        right_line_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height

            win_xleft_low = max(leftx_current - margin, 0)
            win_xleft_high = min(leftx_current + margin, int(midpoint*0.6))

            # Restrict right window to the right side of the frame
            win_xright_low = max(rightx_current - margin, int(midpoint * 1.4))
            win_xright_high = min(rightx_current + margin, binary_warped.shape[1])

            # Draw the windows on the visualization image (Optional)
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_line_inds.append(good_left_inds)
            right_line_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_line_inds = np.concatenate(left_line_inds)
        right_line_inds = np.concatenate(right_line_inds)

        leftx = nonzerox[left_line_inds]
        lefty = nonzeroy[left_line_inds]
        rightx = nonzerox[right_line_inds]
        righty = nonzeroy[right_line_inds]

        return leftx, lefty, rightx, righty, out_img

    def fit_polynomial(self, binary_warped):
        hist, img = self.histogram(binary_warped)
        leftx, lefty, rightx, righty, out_img = self.sliding_window(binary_warped, hist)
        # if len(leftx) == 0 or len(rightx) == 0:
        #   return None, None, out_img, None, None, None, None, img
        left_fit = None
        if len(leftx) != 0:
            left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = None
        if len(rightx) != 0:
            right_fit = np.polyfit(righty, rightx, 2)
        return left_fit, right_fit, out_img, leftx, lefty, rightx, righty, img

    def draw_lane(self, original_image, binary_warped, Minv, aiming_point):
        left_fit = self.left_line.best_fit
        right_fit = self.right_line.best_fit
        average_fit = self.average_line.best_fit

        # Generate y-values for plottings
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        result = original_image.copy()

        left_fitx = None
        right_fitx = None
        average_fitx = None
        if left_fit is None and right_fit is None:
            return original_image

        if left_fit is not None:
            # Calculate x-values for each y-value using the polynomial coefficients
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            # Create points for the line
            left_line_pts = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            # Warp the lane line back onto the original image
            left_line_pts = cv2.perspectiveTransform(left_line_pts, Minv)
            # Drawing
            cv2.polylines(result, [np.int32(left_line_pts)], isClosed=False, color=(255, 0, 0), thickness=10)
        if right_fit is not None:
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
            right_line_pts = np.array([np.transpose(np.vstack([right_fitx, ploty]))])
            right_line_pts = cv2.perspectiveTransform(right_line_pts, Minv)
            cv2.polylines(result, [np.int32(right_line_pts)], isClosed=False, color=(0, 0, 255), thickness=10)
        if left_fit is not None and right_fit is not None:
            average_fitx = average_fit[0] * ploty ** 2 + average_fit[1] * ploty + average_fit[2]
            average_line_pts = np.array([np.transpose(np.vstack([average_fitx, ploty]))])
            average_line_pts = cv2.perspectiveTransform(average_line_pts, Minv)
            cv2.polylines(result, [np.int32(average_line_pts)], isClosed=False, color=(0, 255, 0), thickness=10)

        # Draw the lane lines onto the original image
        point = np.array([[[aiming_point['y'], aiming_point['x']]]], dtype=np.float32)
        point = cv2.perspectiveTransform(point, Minv)
        cv2.circle(result, (int(point[0][0][0]), int(point[0][0][1])), radius=5, color=(0, 255, 255), thickness=-1)
        return result

    def get_perspective_transform(self, image):
        height, width = image.shape[:2]
        if self.is_simu:
            self.top_left = [170, 300]
            self.top_right = [450, 300]
            self.bottom_right = [width * 1, height]
            self.bottom_left = [width * 0.2, height]
        else:
            self.top_left = [width * 0.4, height * 0.7]
            self.top_right = [width * 0.6, height * 0.7]
            self.bottom_right = [width * 0.8, height * 0.95]
            self.bottom_left = [width * 0.2, height * 0.95]
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
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)  # For inverse transform

        return M, Minv

    def warp_image(self, image, M):
        height, width = image.shape[:2]
        warped = cv2.warpPerspective(image, M, (width, height))
        return warped

    def process_frame(self, frame):
        M, Minv = self.get_perspective_transform(frame)
        warped = self.warp_image(frame, M)
        binary_warped = self.threshold_image(warped)
        binary_unwarped = self.warp_image(binary_warped, Minv)
        # cv2.imshow('Binary Unwarped', binary_unwarped)
        # cv2.imshow('Binary Warped', binary_warped)
        left_fit, right_fit, out_img, leftx, lefty, rightx, righty, histImg = self.fit_polynomial(binary_warped)
        # cv2.imshow('Sliding Window', out_img)

        self.left_line.add_fit(left_fit)
        self.right_line.add_fit(right_fit)

        left_fit = self.left_line.best_fit
        right_fit = self.right_line.best_fit

        # Returns a point of the average polynomial at a fixed X value in terms of X in the image space
        aiming_point = {'x': self.distance_to_aim, 'y': 0}
        if left_fit is not None and right_fit is not None:
            self.average_line.add_fit(np.mean([left_fit, right_fit], axis=0))
            aiming_point['y'] = self.average_line.best_fit[0] * aiming_point['x'] ** 2 + \
                self.average_line.best_fit[1] * aiming_point['x'] + self.average_line.best_fit[2]
        elif left_fit is not None:
            aiming_point['y'] = left_fit[0] * aiming_point['x'] ** 2 + \
                left_fit[1] * aiming_point['x'] + left_fit[2] + self.min_offset
        elif right_fit is not None:
            aiming_point['y'] = right_fit[0] * aiming_point['x'] ** 2 + \
                right_fit[1] * aiming_point['x'] + right_fit[2] - self.min_offset

        result = self.draw_lane(frame, binary_warped, Minv, aiming_point)

        return result, aiming_point['y']


if __name__ == '__main__':
    # Open the video file
    # cap = cv2.VideoCapture('test2.mp4')
    # open an image with cv2
    image = cv2.imread('test1.jpg')
    # cv2.imshow("Processed Image", image)
    cv2.waitKey(1)
