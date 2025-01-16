import cv2
import numpy as np
import src.utils.lane_detection.preprocessing as prep
import src.utils.lane_detection.polynomial_fit as fit

def process(image):
    hsvImage = prep.hsv(image)
    cannyFrame = prep.canny(hsvImage, 50, 150)
    #cv2.imshow('canny', cannyFrame)
    warpedFrame, inverseM = prep.warpImage(cannyFrame)
    
    osovina = 300 # 300 piksela, inace 0.2m

    res = fit.lineFit(warpedFrame)
    #cv2.imshow('Sliding Windows', res['outputImage'])
    #cv2.waitKey(0)

    leftFit = res['leftFit']
    rightFit = res['rightFit']
    nonzeroX = res['nonzeroX']
    nonzeroY = res['nonzeroY']
    leftLaneIndicies = res['leftLaneIndicies']
    rightLaneIndicies = res['rightLaneIndicies']

    leftCurve, rightCurve = fit.calculateCurve(warpedFrame, leftLaneIndicies, rightLaneIndicies, nonzeroX, nonzeroY)
    averageCurve = (leftCurve+rightCurve)/2.0
    if leftCurve > rightCurve:
        angle = fit.calculate_steering_angle(osovina, averageCurve, False)
    else:
        angle = fit.calculate_steering_angle(osovina, averageCurve, True)
    return angle
