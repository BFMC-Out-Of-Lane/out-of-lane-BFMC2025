from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    CurrentSpeed, 
    SpeedMotor,
    UltrasonicSensors, 
    mainCamera,
    trafficSigns)
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

        self.current_speed = "0"
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.subscribers = {}

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
            self._handle_ultrasonic_message()
            self._handle_traffic_signs_message()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["UltrasonicSensors"] = messageHandlerSubscriber(
            self.queuesList, UltrasonicSensors, "lastonly", True)
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

    def _handle_traffic_signs_message(self):
        """Handles the traffic signs, i.e.: decides if the vehicle should slow down, stop or continue"""
        try:
            traffic_signs_message = self.subscribers["trafficSigns"].receive()
            if traffic_signs_message:
                detected_clss = str(traffic_signs_message["clss"])
                pixels_distance = int(traffic_signs_message["pixels_distance"])
                if self.debugging:
                    self.logging.info(f'{{"threadDecisionMaker": {{ "Object detected": {detected_clss}, "Distance object-car": {pixels_distance}}} }}')

                if detected_clss == "stop" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    self.speed_sender.send("0")
                    return
                
                if detected_clss == "crosswalk" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    self.speed_sender.send("0")
                    return
                
                if detected_clss == "highway-end" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    pass

                if detected_clss == "highway-entry" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    pass

                if detected_clss == "one-way" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    pass

                if detected_clss == "parking" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    pass

                if detected_clss == "no-entry" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    pass

                if detected_clss == "roundabout" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    pass

                if detected_clss == "priority" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    pass

        except Exception as e:
            print(e)