
import math

import cv2

from ultralytics.solutions.solutions import BaseSolution
from ultralytics.utils.plotting import Annotator, colors

class DistanceCalculation(BaseSolution):
    """
    A class to calculate distance between two objects in a real-time video stream based on their tracks.

    This class extends BaseSolution to provide functionality for selecting objects and calculating the distance
    between them in a video stream using YOLO object detection and tracking.

    Attributes:
        annotator (Annotator): An instance of the Annotator class for drawing on the image.
        boxes (List[List[float]]): List of bounding boxes for detected objects.
        track_ids (List[int]): List of track IDs for detected objects.
        clss (List[int]): List of class indices for detected objects.
        names (List[str]): List of class names that the model can detect.
        centroids (List[List[int]]): List to store centroids of selected bounding boxes.

    Methods:
        mouse_event_for_distance: Handles mouse events for selecting objects in the video stream.
        calculate: Processes video frames and calculates the distance between selected objects.

    Examples:
        >>> distance_calc = DistanceCalculation()
        >>> frame = cv2.imread("frame.jpg")
        >>> processed_frame = distance_calc.calculate(frame)
        >>> cv2.imshow("Distance Calculation", processed_frame)
        >>> cv2.waitKey(0)
    """

    def __init__(self, **kwargs):
        """Initializes the DistanceCalculation class for measuring object distances in video streams."""
        super().__init__(**kwargs)

        self.centroids = []  # Initialize empty list to store centroids

        self.fixed_point = (0, 0)  # Initialize fixed point for distance calculation
        self.pixels_distance = 0  # Initialize distance in pixels
        self.detected_class = None  # Initialize detected class

        """
        Handles mouse events to select regions in a real-time video stream for distance calculation.

        Args:
            event (int): Type of mouse event (e.g., cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN).
            x (int): X-coordinate of the mouse pointer.
            y (int): Y-coordinate of the mouse pointer.
            flags (int): Flags associated with the event (e.g., cv2.EVENT_FLAG_CTRLKEY, cv2.EVENT_FLAG_SHIFTKEY).
            param (Dict): Additional parameters passed to the function.

        Examples:
            >>> # Assuming 'dc' is an instance of DistanceCalculation
            # >>> cv2.setMouseCallback("window_name", dc.mouse_event_for_distance)
        """
        # if event == cv2.EVENT_LBUTTONDOWN:
        #     self.left_mouse_count += 1
        #     if self.left_mouse_count <= 2:
        #         for box, track_id in zip(self.boxes, self.track_ids):
        #             if box[0] < x < box[2] and box[1] < y < box[3] and track_id not in self.selected_boxes:
        #                 self.selected_boxes[track_id] = box

        # elif event == cv2.EVENT_RBUTTONDOWN:
        #     self.selected_boxes = {}
        #     self.left_mouse_count = 0

    def calculate(self, im0):
        """
        Processes a video frame and calculates the distance between two selected bounding boxes.

        This method extracts tracks from the input frame, annotates bounding boxes, and calculates the distance
        between two user-selected objects if they have been chosen.

        Args:
            im0 (numpy.ndarray): The input image frame to process.

        Returns:
            (numpy.ndarray): The processed image frame with annotations and distance calculations.

        Examples:
            >>> import numpy as np
            # >>> from ultralytics.solutions import DistanceCalculation
            >>> dc = DistanceCalculation()
            >>> frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            >>> processed_frame = dc.calculate(frame)
        """
        self.annotator = Annotator(im0, line_width=self.line_width)  # Initialize annotator
        self.extract_tracks(im0)  # Extract tracks

        # Define el punto fijo en el centro de la imagen
        self.fixed_point = (im0.shape[1] // 2, im0.shape[0] // 2)
        # Define el punto fijo en el medio de la imagen pero en la parte inferior
        # self.fixed_point = (im0.shape[1] // 2, im0.shape[0] - 1)

        # Itera sobre las cajas delimitadoras, IDs de pista y clases
        for box, track_id, cls in zip(self.boxes, self.track_ids, self.clss):
            self.annotator.box_label(box, color=colors(int(cls), True), label=self.names[int(cls)])

            # Calcula el centro de la caja delimitadora
            box_center = (int((box[0] + box[2]) // 2), int((box[1] + box[3]) // 2))

            # Calcula la distancia en píxeles entre el centro de la caja y el punto fijo
            self.pixels_distance = math.sqrt(
                (box_center[0] - self.fixed_point[0]) ** 2 + (box_center[1] - self.fixed_point[1]) ** 2
            )

            # Anota la distancia en la imagen
            # self.annotator.plot_distance_and_line(self.pixels_distance, [box_center, self.fixed_point])

            self.detected_class = self.names[int(cls)]
            # print(f"Objeto detectado: {self.names[int(cls)]}, Distancia en píxeles: {self.pixels_distance}")
            # Salir del bucle después de procesar el primer objeto
            break

        # self.display_output(im0)  # Muestra la salida con la función de la clase bas

        return im0, self.detected_class, self.pixels_distance  # Devuelve la imagen procesada para más uso


