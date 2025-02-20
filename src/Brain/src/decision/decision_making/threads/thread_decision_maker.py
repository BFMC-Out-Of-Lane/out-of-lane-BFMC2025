from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, LaneKeeping,
                                            SpeedMotor, SteerMotor,
                                            UltrasonicSensors, mainCamera,
                                            serialCamera, trafficSigns,
                                            WarningSignal)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber

MIN_DISTANCE_TO_STOP = 200
MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN = 250


class threadDecisionMaker(ThreadWithStop):
    """This thread handles decision_making.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.onHighwayLane = False
        self.pedestrianCrossing = False
        self.onStopSignal = False
        self.onStopLight = False

        self.current_speed = "0"
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        self.signal_sender = messageHandlerSender(self.queuesList, WarningSignal)
        self.subscribers = {}

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
            #self._handle_ultrasonic_message()
            self._handle_lane_keeping_message()
            self._handle_traffic_signs_message()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["UltrasonicSensors"] = messageHandlerSubscriber(
            self.queuesList, UltrasonicSensors, "lastonly", True)
        self.subscribers["LaneKeeping"] = messageHandlerSubscriber(self.queuesList, LaneKeeping, "lastonly", True)
        self.subscribers["trafficSigns"] = messageHandlerSubscriber(self.queuesList, trafficSigns, "lastonly", True)

    def _handle_ultrasonic_message(self):
        """Handles the ultrasonic message, i.e.: decides if the vehicle should slow down, stop or continue"""

        ultrasonic_message = self.subscribers["UltrasonicSensors"].receive()
        if ultrasonic_message:
            current_speed = int(self.current_speed)
            front = int(ultrasonic_message["front"])
            rear = int(ultrasonic_message["rear"])
            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "CurrentSpeed": {current_speed}, "UltrasonicSensors": {ultrasonic_message}}} }}'
                )

            if current_speed > 0:
                if front < MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send("0")  # stop the vehicle if front distance is less than 10 cm
                elif current_speed > front - MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send(str(front - MIN_DISTANCE_TO_STOP))
            elif current_speed < 0:
                if rear < MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send("0")  # stop the vehicle if front distance is less than 10 cm
                elif abs(current_speed) > rear - MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send(str(-(rear - MIN_DISTANCE_TO_STOP)))

    def _handle_lane_keeping_message(self):
        """Handles the lane keeping message, i.e.: decides the steering angle"""

        self.lane_offset = self.subscribers["LaneKeeping"].receive()
        if self.lane_offset:
            steering = self.lane_offset * 3  # TODO: Add controller
            if steering < -230:
                steering = -230
            elif steering > 230:
                steering = 230

            self.steer_sender.send(str(steering))

    def _handle_traffic_signs_message(self):
        """Handles the traffic signs, i.e.: decides if the vehicle should slow down, stop or continue"""
        try:
            traffic_signs_message = self.subscribers["trafficSigns"].receive()
            if traffic_signs_message:
                detected_clss = str(traffic_signs_message["clss"])
                pixels_distance = int(traffic_signs_message["pixels_distance"])
                #if self.debugging:
                #    self.logging.info(
                #        f'{{"threadDecisionMaker": {{ "Object detected": {detected_clss}, "Distance object-car": {pixels_distance}}} }}')

                if detected_clss == "stop":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"stop", "WarningID": 20})
                    self.onStopSignal = True
                    
                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": {detected_clss}}} }}')
                    return
                else:
                    self.onStopSignal = False

                if detected_clss == "crosswalk":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"crosswalk", "WarningID": 4})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": {detected_clss}}} }}')
                    return

                if detected_clss == "highway-end":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"highway_exit", "WarningID": 7})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "highway-entry":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"highway_entrance", "WarningID": 6})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "one-way":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"oneway", "WarningID": 9})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "parking":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"parking", "WarningID": 10})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "no-entry":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"forbidden", "WarningID": 5})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "roundabout":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"roundabaout", "WarningID": 15})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "priority":
                    self.signal_sender.send({"WarningName":"priority", "WarningID": 13})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return
                
                if detected_clss == "pedestrian-doll":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"ped_on_crosswalk", "WarningID": 12})
                    self.pedestrianCrossing = True

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return
                else:
                    self.pedestrianCrossing = False
                
                if detected_clss == "red-light":
                    self.speed_sender.send("0")
                    self.signal_sender.send({"WarningName":"red-light", "WarningID": 22})
                    self.onStopLight = True

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return
                else:
                    self.onStopLight = False
                
                if detected_clss == "yellow-light":
                    #self.speed_sender.send("10")
                    self.signal_sender.send({"WarningName":"yellow-light", "WarningID": 23})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return
                
                if detected_clss == "green-light":
                    self.signal_sender.send({"WarningName":"green-light", "WarningID": 24})

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return
                
                # if detected_clss == "priority" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                #     pass

        except Exception as e:
            print(e)
