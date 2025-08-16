import cv2
# from ultralytics import YOLO
from src.utils.visionManager.modelMOD import YOLO


class ObjectDetectionProcessor:

    def __init__(self, yolo_path, conf=0.5):
        # Load the YOLO model
        self.model_path = yolo_path
        self.model = YOLO(self.model_path)  # Load the YOLO model with specified confidence threshold
        self.class_names = self.model.names  # Get class names

        # Average real-world dimensions of a stop sign (in meters)
        self.real_width = 0.06  # Stop sign: 6 cm actual width
        self.focal_length = 480  # Webcam focal length in pixels

    def calculate_distance(self, bbox_width):
        """ Calculates the distance to the stop sign based on bounding box width. """
        distance_m = (self.real_width * self.focal_length) / bbox_width
        return distance_m * 100  # Convert to centimeters

    def process_image(self, cv_image):
        """
        Processes the input image to detect stop signs and display their distance.
        """
        # Perform inference with YOLO
        results = self.model(cv_image)

        objects_distance_dict = {} 

        # Extract detections and draw bounding boxes
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # Mover a CPU antes de convertir a NumPy
            confidences = result.boxes.conf.cpu().numpy()  # Mover a CPU antes de convertir a NumPy
            labels = result.boxes.cls.cpu().numpy().astype(int)  # Mover a CPU antes de convertir a NumPy

            # valid_distance = True  # Default flag for distance validation

            for i, bbox in enumerate(boxes):
                if confidences[i] < 0.5:  # Ignore detections with low confidence
                    continue

                class_name = self.class_names[labels[i]]  # Get the detected class name

                bbox = bbox.astype(int)  # Convert bbox coordinates to integers
                bbox_width = bbox[2] - bbox[0]  # Calculate bounding box width
                distance_cm = self.calculate_distance(bbox_width)  # Compute estimated distance

                # Calculate the center of the bounding box
                center_x = int((bbox[0] + bbox[2]) / 2)
                center_y = int((bbox[1] + bbox[3]) / 2)

                # Añadir un array con los detalles al diccionario
                objects_distance_dict[class_name] = [center_x, center_y]
                # print(objects_distance_dict)
                # print(f"Detected {class_name} at {distance_cm:.2f} cm, center at ({center_x}, {center_y})")

        # return cv_image, objects_distance_dict  # Return the processed image with annotations
        return objects_distance_dict  # Return the processed image with annotations
