from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.visionManager.distanceCalculator import distanceCalculator
import cv2
import numpy as np
import base64

class threadTrafficSigns(ThreadWithStop):
    """This thread handles TrafficSigns.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        super(threadTrafficSigns, self).__init__()

    def run(self):
        while self._running:
            self._handle_camera_frames()
            pass

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.cameraSubscriber = messageHandlerSubscriber(self.queuesList, mainCamera, "lastonly", True)
        pass

    def _handle_camera_frames(self):
        """Handles the camera frames, i.e.: detects traffic signs in the image"""
        camera_message = self.cameraSubscriber.receive()
        if camera_message:
            image_data = base64.b64decode(camera_message)
            np_arr = np.frombuffer(image_data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self._detect_traffic_signs(image)
        pass

    def _detect_traffic_signs(self, image):
        """Detects traffic signs in the image"""

        cap = cv2.VideoCapture(image)
        assert cap.isOpened(), "Error reading video file"
        w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))

        classes_detected = [11] # stop sign

        # Init distance-calculation obj
        distance = DistanceCalculationMOD(model="yolov8n_ncnn_model",
                                        show=True,
                                        conf = 0.3,
                                        # classes = classes_detected,
                                        vid_stride=10,
                                        stream_buffer=True,
                                        # max_det=1,
                                        save=True,
                                        )

        while cap.isOpened():
            success, im0 = cap.read()
            if not success:
                print("Video frame is empty or video processing has been successfully completed.")
                break
            im0 = distance.calculate(im0)

            if self.debugging:
                self.logging.info(
                    f'{{"threadTrafficSigns": {{ "Obj detected": {distance.clss}, "Distance object-car": {distance.pixels_distance}}} }}'
                )

        cap.release()
        #cv2.destroyAllWindows()

        pass