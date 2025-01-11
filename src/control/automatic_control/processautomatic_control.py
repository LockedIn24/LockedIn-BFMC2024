if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.control.automatic_control.threads.threadautomatic_control import threadautomatic_control

class processautomatic_control(WorkerProcess):
    """This process handles automatic_control.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processautomatic_control, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processautomatic_control, self).run()

    def _init_threads(self):
        """Create the automatic_control Publisher thread and add to the list of threads."""
        automatic_controlTh = threadautomatic_control(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(automatic_controlTh)
