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
        # yolo11n_best_traffic_signs_V2.pt
        # yolov8n.pt
        self.distanceCalculator = DistanceCalculation(model=r"/home/ool/out-of-lane-BFMC2025/src/Brain/src/vision/TrafficSigns/yolo11n_best_traffic_signs_V2.pt",
                                                    conf = 0.3,
                                                    # classes = [11],
                                                    vid_stride=10,
                                                    # stream_buffer= True,
                                                    # max_det=1,
                                                    )
        self.distanceToObject = {}
        self.signsSender = messageHandlerSender(self.queuesList, trafficSigns) # Send the detected signs to the decision maker

    def run(self):
        send = True
        while self._running:
            try:
                self._handle_camera_frames()
            except Exception as e:
                print(e)
            
            if send:
                self.signsSender.send(self.distanceToObject)

            send = not send

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
            self._detect_traffic_signs(image)

    def _detect_traffic_signs(self, image):
        """Detects traffic signs in the image"""

        try:
            if self.isDetectorActive:
                    # processed_frame, detected_class, self.distanceToObject = self.distanceCalculator.calculate(image)
                    detected_class_number, self.distanceToObject = self.distanceCalculator.calculate(image)

                    if detected_class_number:
                        detected_class_name = self._get_class_name(detected_class_number) # Asignar nombres de clase basados en detected_class
                    else:
                        detected_class_name = ""

                    self.distanceToObject = {
                        "clss": detected_class_name,
                        "pixels_distance": self.distanceToObject
                        }

                    if self.debugging:
                        self.logging.info(f'{{"threadTrafficSigns": {{ "Object detected": {self.distanceToObject["clss"]}, "Distance object-car": {self.distanceToObject["pixels_distance"]}}} }}')
        except Exception as e:
            print(e)

    def _get_class_name(self, detected_class):
        """Returns the class name based on the detected class integer value"""
        match detected_class[0]:
            case 0.0:
                return "crosswalk"
            case 1.0:
                return "highway-end"
            case 2.0:
                return "highway-entry"
            case 3.0:
                return "no-entry"
            case 4.0:
                return "one-way"
            case 5.0:
                return "parking"
            case 6.0:
                return "priority"
            case 7.0:
                return "roundabout"
            case 8.0:
                return "stop"
            case _:
                return ""