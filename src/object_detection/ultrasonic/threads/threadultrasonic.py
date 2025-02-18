from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender

from src.utils.messages.allMessages import Ultrasonic

import gpiod
import time

class threadultrasonic(ThreadWithStop):
    """This thread handles ultrasonic.
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
        self.ultrasonicSender = messageHandlerSender(self.queuesList, Ultrasonic)
        self.subscribe()
        
        self.signalPin = 17
        self.chip = gpiod.Chip('gpiochip0')
        self.line = self.chip.get_line(self.signalPin)
        # Request the line for input and enable edge detection
        self.line.request(consumer='edge_detection', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
        
        super(threadultrasonic, self).__init__()
        
    # Callback function to run when the pin state changes
    def signal_callback(self, value):
        if value == 1:
            #self.ultrasonicSender.send(True)
            self.syncAutomaticSerial.set()
        else:
            self.ultrasonicSender.send(False)
            self.syncAutomaticSerial.set()

    def run(self):
        try:
            last_value = self.line.get_value()  # Initialize last value
            while self._running:
                # Wait for an event on the line (either rising or falling edge)
                self.line.event_wait()

                 # When an event occurs, read the current value
                value = self.line.get_value()
                
                
                if value != last_value:
                    last_value = value
                    # Detect if it's a rising or falling edge
                    if value == 1:
                        self.signal_callback(1)  # Rising edge
                    else:
                        self.signal_callback(0)  # Falling edge
            
                time.sleep(0.1)  # Sleep to reduce CPU usage

        except KeyboardInterrupt:
            print("\nExiting...")

            # Cleanup: Release the GPIO line
            self.line.release()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        pass
