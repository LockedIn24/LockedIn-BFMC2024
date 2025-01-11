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
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


class threadautomatic_control(ThreadWithStop):
    """This thread handles automatic_control.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.subscribe()

        self.speedSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steerSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.drivingModeSender = messageHandlerSender(self.queuesList, DrivingMode)
        self.klSender = messageHandlerSender(self.queuesList, Klem)

        super(threadautomatic_control, self).__init__()

    def run(self):
        time.sleep(1)
        self.klSender.send("30")
        # self.drivingModeSender.send("auto")
        # self.speedSender.send("25")
        while self._running:
            try:
                # TODO: Speed receive
                radiusRecv = self.radiusSubscriber.receive()
                if radiusRecv is not None:
                    steerValue = self.calculate_steering_angle(0.14, radiusRecv)
                    self.steerSender.send(str(int(steerValue)))
                    self.speedSender.send("25")
            except Exception as e:
                print(e)

    def subscribe(self):
        """Subscribes to the messages you are interested in"""
        self.speedSubscriber = messageHandlerSubscriber(self.queuesList, SpeedLane, "lastonly", True)
        self.radiusSubscriber = messageHandlerSubscriber(self.queuesList, RadiusLane, "lastonly", True)
        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastonly", True)

    def calculate_steering_angle(self, wheelbase, radius_of_curvature, max_steering_angle=25):
        # Calculate the steering angle in radians
        steering_angle_radians = math.atan(wheelbase / radius_of_curvature)
        # Convert the angle to degrees for easier interpretation
        steering_angle_degrees = math.degrees(steering_angle_radians)
        # Ensure the steering angle does not exceed the maximum allowed angle
        if steering_angle_degrees > max_steering_angle:
            steering_angle_degrees = max_steering_angle
        elif steering_angle_degrees < -max_steering_angle:
            steering_angle_degrees = -max_steering_angle

        # Map the angle from -25 to 25 degrees to -250 to 250
        mapped_angle = (steering_angle_degrees / max_steering_angle) * 250
        return mapped_angle
