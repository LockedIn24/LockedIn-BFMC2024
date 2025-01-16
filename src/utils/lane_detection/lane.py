import cv2
import numpy as np

import src.utils.lane_detection.polynomial_fit as fit
import src.utils.lane_detection.preprocessing as prep


def process(image):
    hsvImage = prep.hsv(image)
    #contrastImg = prep.increase_contrast(hsvImage)
    cannyFrame = prep.canny(hsvImage, 50, 150)
    warpedFrame, inverseM = prep.warpImage(cannyFrame)
    shaftPixels = 300
    res = fit.lineFit(warpedFrame)

    leftFit = res['leftFit']
    rightFit = res['rightFit']
    nonzeroX = res['nonzeroX']
    nonzeroY = res['nonzeroY']
    leftLaneIndicies = res['leftLaneIndicies']
    rightLaneIndicies = res['rightLaneIndicies']

    leftCurve, rightCurve = fit.calculateCurve(image, leftLaneIndicies, rightLaneIndicies, nonzeroX, nonzeroY)
    if leftCurve > rightCurve:
        angle = fit.calculate_steering_angle(shaftPixels, (leftCurve+rightCurve)/2, False)
    else:
        angle = fit.calculate_steering_angle(shaftPixels, (leftCurve + rightCurve) / 2, True)

    return angle
