import math
import math
import cv2
import numpy as np

def lineFit(image):
    height = image.shape[0]
    # Take a histogram of the bottom half of the image
    histogram = np.sum(image[int(height/2):,:], axis=0)

    midpoint = int(histogram.shape[0]/2)

    leftPeakXBase = np.argmax(histogram[:midpoint])
    rightPeakXBase = np.argmax(histogram[midpoint:]) + midpoint

    leftPeakXCurrent = leftPeakXBase
    rightPeakXCurrent = rightPeakXBase

    numOfWindows = 9
    windowHeight = int(height/numOfWindows)
    windowMargin = 100
    minpix = 50

    # Find the coordinates of all nonzero pixels in the image
    nonzero = image.nonzero()
    nonzeroY = np.array(nonzero[0])
    nonzeroX = np.array(nonzero[1])

    leftLaneIndicies = []
    rightLaneIndicies = []

    for window in range(numOfWindows):
        # Calculate window boundaries
        windowYLow = height - (window+1)*windowHeight
        windowYHigh = height - window*windowHeight
        windowXLeftLow = leftPeakXCurrent - windowMargin
        windowXLeftHigh = leftPeakXCurrent + windowMargin
        windowXRightLow = rightPeakXCurrent - windowMargin
        windowXRightHigh = rightPeakXCurrent + windowMargin

        # Identify the nonzero pixels in x and y within the window
        goodLeftIndicies = ((nonzeroY >= windowYLow) & (nonzeroY < windowYHigh) & (nonzeroX >= windowXLeftLow) & (nonzeroX < windowXLeftHigh)).nonzero()[0]
        goodRightIndicies = ((nonzeroY >= windowYLow) & (nonzeroY < windowYHigh) & (nonzeroX >= windowXRightLow) & (nonzeroX < windowXRightHigh)).nonzero()[0]

        leftLaneIndicies.append(goodLeftIndicies)
        rightLaneIndicies.append(goodRightIndicies)

        if len(goodLeftIndicies) > minpix:
            leftPeakXCurrent = int(np.mean(nonzeroX[goodLeftIndicies]))
        if len(goodRightIndicies) > minpix:
            rightPeakXCurrent = int(np.mean(nonzeroX[goodRightIndicies]))

    leftLaneIndicies = np.concatenate(leftLaneIndicies)
    rightLaneIndicies = np.concatenate(rightLaneIndicies)

    # Extract left and right line pixel positions
    leftX = nonzeroX[leftLaneIndicies]
    leftY = nonzeroY[leftLaneIndicies]
    rightX = nonzeroX[rightLaneIndicies]
    rightY = nonzeroY[rightLaneIndicies]

    # Add checks for empty vectors before fitting polynomials
    if len(leftX) > 0 and len(leftY) > 0:
        leftFit = np.polyfit(leftY, leftX, 2)
    else:
        leftFit = [0, 0, 0]  # Default to a horizontal line if no valid left lane points

    if len(rightX) > 0 and len(rightY) > 0:
        rightFit = np.polyfit(rightY, rightX, 2)
    else:
        rightFit = [0, 0, 0]  # Default to a horizontal line if no valid right lane points

    ret = {}
    ret['leftFit'] = leftFit
    ret['rightFit'] = rightFit
    ret['nonzeroX'] = nonzeroX
    ret['nonzeroY'] = nonzeroY
    ret['leftLaneIndicies'] = leftLaneIndicies
    ret['rightLaneIndicies'] = rightLaneIndicies

    return ret

def calculateCurve(image, leftLaneIndicies, rightLaneIndicies, nonzeroX, nonzeroY):
    yEval = image.shape[0] - 1  

    # Extract left and right line pixel positions
    leftX = nonzeroX[leftLaneIndicies]
    leftY = nonzeroY[leftLaneIndicies]
    rightX = nonzeroX[rightLaneIndicies]
    rightY = nonzeroY[rightLaneIndicies]

    if len(leftX) > 0 and len(leftY) > 0:
        leftFitCurve = np.polyfit(leftY, leftX, 2)
    else:
        leftFitCurve = [0, 0, 0]  # Default values if no left lane points are found

    if len(rightX) > 0 and len(rightY) > 0:
        rightFitCurve = np.polyfit(rightY, rightX, 2)
    else:
        rightFitCurve = [0, 0, 0]  # Default values if no right lane points are found

    # Calculate the radius of curvature
    leftCurveRadius = ((1 + (2 * leftFitCurve[0] * yEval + leftFitCurve[1])**2)**1.5) / np.absolute(2 * leftFitCurve[0])
    rightCurveRadius = ((1 + (2 * rightFitCurve[0] * yEval + rightFitCurve[1])**2)**1.5) / np.absolute(2 * rightFitCurve[0])
    
    averageCurveRadius = (leftCurveRadius + rightCurveRadius) / 2

    if averageCurveRadius > 3000:
        leftCurveRadius = math.inf        
        rightCurveRadius = math.inf
    return leftCurveRadius, rightCurveRadius



def calculate_steering_angle(wheelbase, radius_of_curvature, inverse, max_steering_angle=25):
    if abs(radius_of_curvature) < 1e-6:
        raise ValueError("Radius of curvature is too small or zero, which is not realistic.")
    steering_angle_radians = math.atan(wheelbase / abs(radius_of_curvature))
    steering_angle_degrees = math.degrees(steering_angle_radians)
    steering_angle_degrees = max(-max_steering_angle, min(max_steering_angle, steering_angle_degrees))
    if steering_angle_degrees == 0:
        return 0.0
    else:
        steering_angle_degrees = steering_angle_degrees + 5.0

    if inverse:
        steering_angle_degrees = -steering_angle_degrees

    steering_angle_degrees = max(-max_steering_angle, min(max_steering_angle, steering_angle_degrees))
    steering_angle_degrees = round(steering_angle_degrees, 0) * 10.0
    return steering_angle_degrees


