if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.ultrasonic.ultrasonic.threads.threadultrasonic import threadultrasonic

class ProcessUltrasonic(WorkerProcess):
    """This process handles ultrasonic.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, syncAutomaticSerial, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.syncAutomaticSerial = syncAutomaticSerial
        super(ProcessUltrasonic, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(ProcessUltrasonic, self).run()

    def _init_threads(self):
        """Create the ultrasonic Publisher thread and add to the list of threads."""
        ultrasonicTh = threadultrasonic(self.queuesList, self.syncAutomaticSerial, self.logging, self.debugging)
        self.threads.append(ultrasonicTh)
