import cv2
import numpy as np

from src.hardware.camera.lane_detection.lane_detection_interface import LaneDetectionInterface


def threshold_image(image):
    # Apply Gaussian blur to the frame to reduce noise
    blurred_frame = cv2.GaussianBlur(image, (5, 5), 0)

    hls = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HLS)
    s_channel = hls[:, :, 2]
    l_channel = hls[:, :, 1]

    white_mask = cv2.inRange(l_channel, 200, 255)
    """
    Yellow mask is not needed for simulator or race track

    lower_yellow = np.array([15, 30, 115])
        upper_yellow = np.array([35, 204, 255])
        yellow_mask = cv2.inRange(hls, lower_yellow, upper_yellow)

    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
     """

    gray = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2GRAY)
    blurred_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    sobelx = cv2.Sobel(blurred_gray, cv2.CV_64F, 1, 0)
    abs_sobelx = np.absolute(sobelx)
    # Avoid division by zero
    max_abs_sobelx = np.max(abs_sobelx)
    if max_abs_sobelx == 0:
        max_abs_sobelx = 1
    scaled_sobel = np.uint8(255 * abs_sobelx / max_abs_sobelx)

    thresh_min = 20
    thresh_max = 100
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 255

    combined_binary = cv2.bitwise_or(white_mask, sxbinary)

    return combined_binary


def histogram(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
    return histogram


def sliding_window(binary_warped, histogram):
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

    margin = 100
    minpix = 50  # Minimum number of pixels found to recenter window

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height

        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin

        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Draw the windows on the visualization image (Optional)
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty, out_img


def fit_polynomial(binary_warped):
    hist = histogram(binary_warped)
    leftx, lefty, rightx, righty, out_img = sliding_window(binary_warped, hist)
    if len(leftx) == 0 or len(rightx) == 0:
        return None, None, out_img, None, None, None, None
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    return left_fit, right_fit, out_img, leftx, lefty, rightx, righty


def draw_lane(original_image, binary_warped, Minv, left_fit, right_fit):
    if left_fit is None or right_fit is None:
        return original_image

    # Generate y-values for plotting
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])

    # Calculate x-values for each y-value using the polynomial coefficients
    try:
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    except TypeError:
        # If the fit fails, just return the original image
        return original_image

    # Create an image to draw the lines on
    warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # Create points for the left and right lines
    left_line_pts = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    right_line_pts = np.array([np.transpose(np.vstack([right_fitx, ploty]))])

    # Warp the lane lines back onto the original image
    left_line_pts = cv2.perspectiveTransform(left_line_pts, Minv)
    right_line_pts = cv2.perspectiveTransform(right_line_pts, Minv)

    # Draw the lane lines onto the original image
    result = original_image.copy()
    cv2.polylines(result, [np.int32(left_line_pts)], isClosed=False, color=(255, 0, 0), thickness=10)
    cv2.polylines(result, [np.int32(right_line_pts)], isClosed=False, color=(0, 0, 255), thickness=10)

    return result


def get_perspective_transform(image):
    height, width = image.shape[:2]
    top_left = [250, 300]
    top_right = [450, 300]
    bottom_right = [width * 1, height]
    bottom_left = [width * 0, height]
    src = np.float32([
        top_left,
        top_right,
        bottom_right,
        bottom_left
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


def warp_image(image, M):
    height, width = image.shape[:2]
    warped = cv2.warpPerspective(image, M, (width, height))
    return warped


def process_frame(frame):
    M, Minv = get_perspective_transform(frame)
    warped = warp_image(frame, M)
    binary_warped = threshold_image(warped)
    left_fit, right_fit, out_img, leftx, lefty, rightx, righty = fit_polynomial(binary_warped)
    result = draw_lane(frame, binary_warped, Minv, left_fit, right_fit)
    return result


class LaneDetector(LaneDetectionInterface):
    def __init__(self):
        pass

    def process_image(self, cv_image):
        return process_frame(cv_image)


if __name__ == '__main__':
    # Open the video file
    # cap = cv2.VideoCapture('test2.mp4')
    # open an image with cv2
    image = cv2.imread('test1.jpg')
    cv2.imshow("Processed Image", image)
    cv2.waitKey(1)

""" 
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        result = process_frame(frame)
        cv2.imshow('Lane Detection', result)
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
 """
