if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import os
import subprocess

from src.decision.decision_making.threads.thread_decision_maker import \
    threadDecisionMaker
from src.templates.workerprocess import WorkerProcess


class processDecisionMaker(WorkerProcess):
    """This process handles decision_making.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processDecisionMaker, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        # set_highest_priority()
        super(processDecisionMaker, self).run()

    def _init_threads(self):
        """Create the decision_making Publisher thread and add to the list of threads."""
        decision_makingTh = threadDecisionMaker(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(decision_makingTh)


def set_highest_priority():
    pid = os.getpid()
    subprocess.run(["sudo", "renice", "-20", "-p", str(pid)], check=True)
