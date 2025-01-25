from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, LaneKeeping,
                                            SpeedMotor, UltrasonicSensors, SteerMotor,
                                            mainCamera, serialCamera)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber

MIN_DISTANCE_TO_STOP = 200


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
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        self.subscribers = {}

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
            self._handle_ultrasonic_message()

            # TODO: This should be tested
            self.lane_offset = self.subscribers["LaneKeeping"].receive()
            if self.lane_offset:
                self.steer_sender.send(str(self.lane_offset * 3))

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["UltrasonicSensors"] = messageHandlerSubscriber(
            self.queuesList, UltrasonicSensors, "lastonly", True)
        self.subscribers["LaneKeeping"] = messageHandlerSubscriber(self.queuesList, LaneKeeping, "lastonly", True)

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
