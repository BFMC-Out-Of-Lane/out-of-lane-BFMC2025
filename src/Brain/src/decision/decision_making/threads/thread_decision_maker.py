from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, SpeedMotor,
                                            UltrasonicSensors, mainCamera)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber

MIN_DISTANCE_UNTIL_BREAK = 25


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
            ultrasonic_message = self.subscribers["UltrasonicSensors"].receive()
            if ultrasonic_message:
                current_speed = int(self.current_speed)
                front = int(ultrasonic_message["front"]) * 10
                rear = int(ultrasonic_message["rear"]) * 10
                if self.debugging:
                    self.logging.info(f"threadDecisionMaker: {ultrasonic_message, current_speed}")

                if current_speed > 0:
                    if front < MIN_DISTANCE_UNTIL_BREAK:
                        self.speed_sender.send("0")  # stop the vehicle if front distance is less than 5 cm
                    elif int(current_speed) > front:
                        self.speed_sender.send(str(front))
                elif current_speed < 0:
                    if rear < MIN_DISTANCE_UNTIL_BREAK:
                        self.speed_sender.send("0")  # stop the vehicle if front distance is less than 5 cm
                    elif current_speed > rear:
                        self.speed_sender.send(str(rear))

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["UltrasonicSensors"] = messageHandlerSubscriber(
            self.queuesList, UltrasonicSensors, "lastonly", True)
