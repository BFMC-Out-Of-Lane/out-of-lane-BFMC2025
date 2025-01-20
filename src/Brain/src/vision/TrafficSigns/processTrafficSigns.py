if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.vision.TrafficSigns.threads.threadTrafficSigns import threadTrafficSigns

class processTrafficSigns(WorkerProcess):
    """This process handles TrafficSigns.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processTrafficSigns, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processTrafficSigns, self).run()

    def _init_threads(self):
        """Create the TrafficSigns Publisher thread and add to the list of threads."""
        TrafficSignsTh = threadTrafficSigns(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(TrafficSignsTh)

# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processCamera.py
if __name__ == "__main__":
    from multiprocessing import Queue, Event
    import time
    import logging
    import cv2
    import base64
    import numpy as np