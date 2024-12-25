from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import UltrasonicSensors, mainCamera
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber


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
        self.suscribers = {}
        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            ultrasonic_message = self.suscribers["UltrasonicSensors"].receive()
            if ultrasonic_message is not None and self.debugging:
                self.logging.info(f"UltrasonicSensors: {ultrasonic_message}")

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.suscribers["UltrasonicSensors"] = messageHandlerSubscriber(
            self.queuesList, UltrasonicSensors, "lastOnly", True)
