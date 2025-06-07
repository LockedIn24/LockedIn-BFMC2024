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

from collections import defaultdict

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
        self.currentSign = -1
        self.signs = defaultdict(int)
        self.stopedBefore = False
        self.isParking = False

        self.speedSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steerSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.drivingModeSender = messageHandlerSender(self.queuesList, DrivingMode)
        self.klSender = messageHandlerSender(self.queuesList, Klem)

        super(threadautomatic_control, self).__init__()

    def run(self):
        time.sleep(0.5)
        self.klSender.send("30")
        self.syncAutomaticSerial.set()
        time.sleep(0.5)
        self.speedSender.send("100")
        self.syncAutomaticSerial.set()
        time.sleep(0.3)
        while self._running:
            try:
                # TODO: Speed receive
                time.sleep(0.01)
                self.syncCameraAutomatic.wait()
                angle = self.radiusSubscriber.receive()
                if angle is not None:
                    self.steerSender.send(str(int(angle)))
                    self.syncAutomaticSerial.set()
                
                time.sleep(0.01)
                self.syncCameraAutomatic.wait()
                self.currentSign = self.signSubscriber.receive()

                if self.currentSign is not None:
                    self.signReaction() 
                    print(f"Sign that i got is {self.currentSign}")
            except Exception as e:
                print(e)
                
    def signReaction(self):

        self.signs[self.currentSign] += 1
        if self.signs[self.currentSign] == 1:
            if self.currentSign == 0:       # Crosswalk
                self.crosswalkSign()
            elif self.currentSign == 1:     # Highway entrance    
                self.highwayEntranceSign()
                time.sleep(4)
                self.highwayExitSign()
            elif self.currentSign == 2:     # Highway exit    
                self.highwayExitSign()
            elif self.currentSign == 3:     # No entry road - delete later??
                pass
            elif self.currentSign == 4:     # One way road    
                pass
            elif self.currentSign == 5:     # Parking    
                self.parkingSign()
            elif self.currentSign == 6:     # Pedestrian - delete later    
                pass
            elif self.currentSign == 7:     # Priority    
                pass
            elif self.currentSign == 8:     # Round-about
                pass
            elif self.currentSign == 9 and not self.stopedBefore:     # Stop
                self.stopSign()
            
            if self.stopedBefore and self.currentSign != 9:
                self.stopedBefore = False

            self.signs.clear()    

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.speedSubscriber = messageHandlerSubscriber(self.queuesList, SpeedLane, "lastonly", True)
        self.radiusSubscriber = messageHandlerSubscriber(self.queuesList, RadiusLane, "lastonly", True)
        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastonly", True)
        self.signSubscriber = messageHandlerSubscriber(self.queuesList, CurrentSign, "lastOnly", True)

    def stopSign(self):
        self.speedSender.send("0")
        self.syncAutomaticSerial.set()
        time.sleep(1)
        self.speedSender.send("150")
        self.syncAutomaticSerial.set()
        self.stopedBefore = True

    def highwayEntranceSign(self):
        self.speedSender.send("300")
        self.syncAutomaticSerial.set()

    def highwayExitSign(self):
        self.speedSender.send("150")
        self.syncAutomaticSerial.set()
    
    def crosswalkSign(self):
        if self.isParking == True:
                self.isParking = False
        time.sleep(1.2)
        self.speedSender.send("50")
        self.syncAutomaticSerial.set()
        time.sleep(3.7)
        self.speedSender.send("100")
        self.syncAutomaticSerial.set()

    def parkingSign(self):
        self.isParking = True
        self.speedSender.send("100")
        self.syncAutomaticSerial.set()
        time.sleep(11.7)
        self.speedSender.send("-100")
        self.syncAutomaticSerial.set()
        time.sleep(0.2)
        self.steerSender.send("-250")
        self.syncAutomaticSerial.set()
        time.sleep(3.8)
        self.steerSender.send("250")
        self.syncAutomaticSerial.set()
        time.sleep(3.5)
        self.speedSender.send("100")
        self.syncAutomaticSerial.set()
        time.sleep(0.2)
        self.steerSender.send("-250")
        self.syncAutomaticSerial.set()
        time.sleep(0.4)
        self.syncAutomaticSerial.set()
        self.speedSender.send("0")
        time.sleep(0.1)
        self.syncAutomaticSerial.set()
        self.steerSender.send("0")
        self.isParking = False
        self.syncAutomaticSerial.set()
