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
    CurrentSign,
    SignSize
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
        self.signSize = 0
        self.stopedBefore = False

        self.speedSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steerSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.drivingModeSender = messageHandlerSender(self.queuesList, DrivingMode)
        self.klSender = messageHandlerSender(self.queuesList, Klem)

        super(threadautomatic_control, self).__init__()

    def run(self):
        self.klSender.send("30")
        self.syncAutomaticSerial.set()
        time.sleep(0.2)
        #self.speedSender.send("150")
        self.syncAutomaticSerial.set()
        time.sleep(0.2)
        while self._running:
            try:
                # TODO: Speed receive
                #time.sleep(0.05)
                #self.syncCameraAutomatic.wait()
                #angle = self.radiusSubscriber.receive()
                #if angle is not None:
                    #self.steerSender.send(str(int(angle)))
                    #self.syncAutomaticSerial.set()
                
                time.sleep(0.05)
                self.syncCameraAutomatic.wait()
                self.currentSign = self.signSubscriber.receive()
                time.sleep(0.05)
                self.signSize = self.signSizeSubscriber.receive()

                if self.currentSign is not None and self.signSize is not None:
                    self.signReaction() 
                    #print(f"Sign that i got is {self.currentSign}")
                    #print(f"Area of that sign is {self.signSize}")
            except Exception as e:
                print(e)
                
    def signReaction(self):
        
        if not self.stopedBefore and self.currentSign == "Stop sign" and self.signSize > 2000:
            self.speedSender.send("0")
            self.syncAutomaticSerial.set()
            time.sleep(1)
            self.speedSender.send("150")
            self.syncAutomaticSerial.set()
            self.stopedBefore = True

        if self.currentSign != "Stop sign":
            self.stopedBefore = False

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.speedSubscriber = messageHandlerSubscriber(self.queuesList, SpeedLane, "lastonly", True)
        self.radiusSubscriber = messageHandlerSubscriber(self.queuesList, RadiusLane, "lastonly", True)
        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastonly", True)
        self.signSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSign, "lastOnly", True)
        self.signSizeSubscriber = messageHandlerSubscriber(self.queuesList, SignSize, "lastOnly", True)
