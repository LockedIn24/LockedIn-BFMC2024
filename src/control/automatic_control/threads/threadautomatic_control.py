import math
import time

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    DrivingMode,
    Klem,
    RadiusLane,
    SpeedLane,
    SpeedMotor,
    SteerMotor,
    CurrentSign
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber


class threadautomatic_control(ThreadWithStop):
    """This thread handles automatic_control.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, syncCameraAutomatic, syncAutomaticSerial, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()
        self.syncAutomaticSerial = syncAutomaticSerial
        self.syncCameraAutomatic = syncCameraAutomatic
        self.counter = 0
        self.currentSign = ""

        self.speedSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steerSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.drivingModeSender = messageHandlerSender(self.queuesList, DrivingMode)
        self.klSender = messageHandlerSender(self.queuesList, Klem)

        super(threadautomatic_control, self).__init__()

    def run(self):
        
        time.sleep(1)
        self.klSender.send("30")
        self.syncAutomaticSerial.set()
        time.sleep(0.5)
        self.speedSender.send("150")
        self.syncAutomaticSerial.set()
        while self._running:
            try:
                time.sleep(0.5)
                # TODO: Speed receive
                self.syncCameraAutomatic.wait()
                angle = self.radiusSubscriber.receive()
                if angle is not None:
                    self.steerSender.send(str(int(angle)))
                    self.syncAutomaticSerial.set()
                
                if self.counter == 10:
                    self.currentSign = self.signSubscriber.receive()
                    self.signReaction() 
                    print(f"Sign that i got is {self.currentSign}")
                    self.counter = 0
                
                self.counter += 1
            except Exception as e:
                print(e)
                
    def signReaction(self):
        
        if self.currentSign == "Stop Sign":
            self.speedSender.send("0")
            self.syncAutomaticSerial.set()
            time.sleep(0.5)
            self.speedSender.send("150")
            self.syncAutomaticSerial.set()

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.speedSubscriber = messageHandlerSubscriber(self.queuesList, SpeedLane, "lastonly", True)
        self.radiusSubscriber = messageHandlerSubscriber(self.queuesList, RadiusLane, "lastonly", True)
        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastonly", True)
        self.signSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSign, "lastOnly", True)
