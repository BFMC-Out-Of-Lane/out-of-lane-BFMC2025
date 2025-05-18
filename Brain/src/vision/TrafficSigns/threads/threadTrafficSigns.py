from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    trafficSigns)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.visionManager.distanceCalculator import DistanceCalculation
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
    # ================================ INIT ===============================================

    def __init__(self, queueList, logging, debugging=False):
        super(threadTrafficSigns, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()

        self.isDetectorActive = True

        #  r"/home/ool/out-of-lane-BFMC2025/src/Brain/src/vision/
        # model="src/vision/TrafficSigns/yolo11n_traffic_objects_V11_ncnn_model/",
        # yolo11n_traffic_objects_V11.pt
        # yolov8n.pt

        self.distanceCalculator = DistanceCalculation(model="src/vision/TrafficSigns/yolo11n_traffic_objects_V11.engine",
                                                      conf=0.8,
                                                      # classes = [0,8],
                                                      vid_stride=10,
                                                      # stream_buffer= True,
                                                      # max_det=1,
                                                      )
        self.distanceToObjectMessage = {}
        # Send the detected signs to the decision maker
        self.signsSender = messageHandlerSender(self.queuesList, trafficSigns)

    def run(self):
        while self._running:
            try:
                self._handle_camera_frames()
            except Exception as e:
                print(e)

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        # self.cameraSubscriber = messageHandlerSubscriber(self.queuesList, mainCamera, "lastonly", True)
        self.cameraSubscriber = messageHandlerSubscriber(self.queuesList, serialCamera, "lastonly", True)
        pass

    # =============================== STOP ================================================
    def stop(self):
        super(threadTrafficSigns, self).stop()

    def start(self):
        super(threadTrafficSigns, self).start()

    def _handle_camera_frames(self):
        """Handles the camera frames, i.e.: detects traffic signs in the image"""
        camera_message = self.cameraSubscriber.receive()
        if camera_message:
            image_data = base64.b64decode(camera_message)
            np_arr = np.frombuffer(image_data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image = cv2.add(image, np.array([30, 30, 30], dtype=np.uint8))
            self._detect_traffic_signs(image)

    def _detect_traffic_signs(self, image):
        """Detects traffic signs in the image"""

        try:
            if self.isDetectorActive:
                detectedObjectDistanceDict = self.distanceCalculator.calculate(image)
                if detectedObjectDistanceDict:
                    # Las variables toman los siguientes valores:
                    # detectedClassName = ["stop-signal", "crosswalk-signal" ]
                    # distanceToObject = [200.0, 100.0]

                    self.distanceToObjectMessage = {
                        "objectDict": detectedObjectDistanceDict,
                    }
                    if self.debugging:
                        self.logging.info(
                            f'{{"threadTrafficSigns": {{ "Object-Distance": {self.distanceToObjectMessage["objectDict"]}}} }}')

                    self.signsSender.send(self.distanceToObjectMessage)
        except Exception as e:
            print(e)
