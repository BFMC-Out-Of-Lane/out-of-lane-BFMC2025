from abc import ABC, abstractmethod


class LaneDetectionInterface(ABC):
    """
    Interface for image processors.
    """
    @abstractmethod
    def process_image(self, cv_image, traffic_sign):
        """
        Processes the image and performs some action (detection of curves, lines, or any model we are going to test).
        :param cv_image: Image in OpenCV format.
        """
        pass
