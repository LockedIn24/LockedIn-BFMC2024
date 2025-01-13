import cv2
import numpy as np

import src.utils.lane_detection.polynomial_fit as fit
import src.utils.lane_detection.preprocessing as prep


def process(image):
    hsvImage = prep.hsv(image)
    #contrastImg = prep.increase_contrast(hsvImage)
    cannyFrame = prep.canny(hsvImage, 50, 150)
    warpedFrame, inverseM = prep.warpImage(cannyFrame)

    res = fit.lineFit(warpedFrame)

    leftFit = res['leftFit']
    rightFit = res['rightFit']
    nonzeroX = res['nonzeroX']
    nonzeroY = res['nonzeroY']
    leftLaneIndicies = res['leftLaneIndicies']
    rightLaneIndicies = res['rightLaneIndicies']

    leftCurve, rightCurve = fit.calculateCurve(image, leftLaneIndicies, rightLaneIndicies, nonzeroX, nonzeroY)
    # angle = fit.calculate_steering_angle(0.2, (leftCurve+rightCurve)/2)
    # print(angle)

    bottomY = image.shape[0] - 1
    bottomXLeft = leftFit[0]*(bottomY**2) + leftFit[1]*bottomY + leftFit[2]
    bottomXRight = rightFit[0]*(bottomY**2) + rightFit[1]*bottomY + rightFit[2]
    vehicleOffset = image.shape[1]/2 - (bottomXLeft + bottomXRight)/2

    metersPerPixelX = 0.35/700
    vehicleOffset *= metersPerPixelX

    effective_radius = (leftCurve + rightCurve) / 2

    result = fit.showResult(image, leftFit, rightFit, inverseM, leftCurve, rightCurve, vehicleOffset)

    return result, effective_radius
