import math
import threading
import time

from src.decision.decision_making.odometer.odometer import Odometer
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, LaneKeeping,
                                            SpeedMotor, SteerMotor,
                                            UltrasonicSensors, WarningSignal,
                                            mainCamera, serialCamera,
                                            trafficSigns)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber

MIN_DISTANCE_TO_STOP = 200
MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN = 250
TIME_AT_STOP_SIGN = 3  # seconds


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
        self.odometer = Odometer()

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
            # self._handle_ultrasonic_message()
            try:
                self._handle_lane_keeping_message()
                self._handle_traffic_signs_message()
            except Exception as e:
                print(e)

            self.odometer.update(int(self.current_speed))

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

        lane_keeping = self.subscribers["LaneKeeping"].receive()
        if lane_keeping and int(self.current_speed) > 0:
            self.cross_tracking_error = lane_keeping["cross_tracking_error"]
            self.deviation = lane_keeping["deviation"]
            if self.cross_tracking_error and self.deviation:
                steering = int(self.deviation + math.atan(1 * self.cross_tracking_error /
                               self.current_speed) * 180 / math.pi) * 10
                if steering < -230:
                    steering = -230
                elif steering > 230:
                    steering = 230

                self.steer_sender.send(str(steering))

    def _handle_traffic_signs_message(self):
        """Handles the traffic signs, i.e.: decides if the vehicle should slow down, stop or continue"""

        def timer_stop_signal(t):
            self.onStopSignal = True
            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": {detected_clss}}} }}')
            self.speed_sender.send("0")
            time.sleep(t)
            self.onStopSignal = False

        traffic_signs_message = self.subscribers["trafficSigns"].receive()
        if traffic_signs_message:
            detected_clss = traffic_signs_message["clss"]
            pixels_distance = traffic_signs_message["pixels_distance"]

            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "Object detected": {detected_clss}, "Distance object-car": {pixels_distance}}} }}')

            if "stop-sign" in detected_clss and not self.onStopSignal:
                self.signal_sender.send(str({"WarningName": "stop", "WarningID": 20}))
                if pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    timer_stop_signal(TIME_AT_STOP_SIGN)

            if "crosswalk-sign" in detected_clss or "crosswalk-road" in detected_clss:
                self.speed_sender.send("100")
                self.signal_sender.send({"WarningName": "crosswalk", "WarningID": 4})

            if "highway-end-sign" in detected_clss:
                self.speed_sender.send("200")
                self.signal_sender.send({"WarningName": "highway_exit", "WarningID": 7})

            if "highway-entry-sign" in detected_clss:
                self.speed_sender.send("400")
                self.signal_sender.send({"WarningName": "highway_entrance", "WarningID": 6})

            if "one-way-sign" in detected_clss:
                self.signal_sender.send({"WarningName": "oneway", "WarningID": 9})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: one-way-sign" }}}}')

            if "parking-sign" in detected_clss:
                self.speed_sender.send("0")
                # TODO: parking method
                self.signal_sender.send({"WarningName": "parking", "WarningID": 10})

            if "no-entry-sign" in detected_clss:
                self.speed_sender.send("0")
                # TODO: Add path plannign logic
                self.signal_sender.send({"WarningName": "forbidden", "WarningID": 5})

            if "roundabout-sign" in detected_clss:
                self.signal_sender.send({"WarningName": "roundabaout", "WarningID": 15})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": "Object detected: roundabout-sign" }}')

            if "priority-sign" in detected_clss:
                self.signal_sender.send({"WarningName": "priority", "WarningID": 13})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": "Object detected: priority-signal }}')

            if "pedestrian" in detected_clss:
                self.speed_sender.send("0")
                self.signal_sender.send({"WarningName": "ped_on_crosswalk", "WarningID": 12})
                self.pedestrianCrossing = True
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": "Object detected: pedestrian }}')
            else:
                self.pedestrianCrossing = False

            if "red-light" in detected_clss:
                self.speed_sender.send("0")
                self.signal_sender.send({"WarningName": "red-light", "WarningID": 22})
                self.onStopLight = True
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: red-light" }}}}')
            else:
                self.onStopLight = False

            if "yellow-light" in detected_clss:
                self.signal_sender.send({"WarningName": "yellow-light", "WarningID": 23})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected:yellow-light" }}}}')

            if "green-light" in detected_clss:
                self.signal_sender.send({"WarningName": "green-light", "WarningID": 24})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: green-light" }}}}')
