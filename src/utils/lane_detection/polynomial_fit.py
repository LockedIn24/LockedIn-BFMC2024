import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

def lineFit(image):
    height, width = image.shape[:2]
    output_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convert to BGR for drawing

    # Take a histogram of the bottom half of the image
    histogram = np.sum(image[int(height / 2):, :], axis=0)

    midpoint = int(histogram.shape[0] / 2)

    leftPeakXBase = np.argmax(histogram[:midpoint])
    rightPeakXBase = np.argmax(histogram[midpoint:]) + midpoint

    leftPeakXCurrent = leftPeakXBase
    rightPeakXCurrent = rightPeakXBase

    numOfWindows = 9
    windowHeight = int(height / numOfWindows)
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
        windowYLow = height - (window + 1) * windowHeight
        windowYHigh = height - window * windowHeight
        windowXLeftLow = leftPeakXCurrent - windowMargin
        windowXLeftHigh = leftPeakXCurrent + windowMargin
        windowXRightLow = rightPeakXCurrent - windowMargin
        windowXRightHigh = rightPeakXCurrent + windowMargin

        # Draw the rectangles on the image
        cv2.rectangle(output_image, (windowXLeftLow, windowYLow), (windowXLeftHigh, windowYHigh), (0, 255, 0), 2)
        cv2.rectangle(output_image, (windowXRightLow, windowYLow), (windowXRightHigh, windowYHigh), (0, 0, 255), 2)

        # Identify the nonzero pixels in x and y within the window
        goodLeftIndicies = ((nonzeroY >= windowYLow) & (nonzeroY < windowYHigh) & (nonzeroX >= windowXLeftLow) & (nonzeroX < windowXLeftHigh)).nonzero()[0]
        goodRightIndicies = ((nonzeroY >= windowYLow) & (nonzeroY < windowYHigh) & (nonzeroX >= windowXRightLow) & (nonzeroX < windowXRightHigh)).nonzero()[0]
        # these are numpy arrays of indices of pixels that are nonzero in each window
            
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
        rightFit = [0, 0, 0]  # Default to a horizontal line if no vali
    
    ret = {}
    ret['leftFit'] = leftFit
    ret['rightFit'] = rightFit
    ret['nonzeroX'] = nonzeroX
    ret['nonzeroY'] = nonzeroY
    ret['leftLaneIndicies'] = leftLaneIndicies
    ret['rightLaneIndicies'] = rightLaneIndicies
    ret['outputImage'] = output_image

    return ret

def calculateCurve(image, leftLaneIndicies, rightLaneIndicies, nonzeroX, nonzeroY):
    yEval = image.shape[0] - 1
   
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

    leftCurveRadius = ((1 + (2 * leftFit[0] * yEval  + leftFit[1])**2)**1.5) / np.absolute(2 * leftFit[0])
    rightCurveRadius = ((1 + (2 * rightFit[0] * yEval  + rightFit[1])**2)**1.5) / np.absolute(2 * rightFit[0])
    
    
    averageCurveRadius = (leftCurveRadius + rightCurveRadius) / 2
    if averageCurveRadius > 3000:
        leftCurveRadius = math.inf
        rightCurveRadius = math.inf
    return leftCurveRadius, rightCurveRadius

def showResult(undistortedFrame, leftFit, rightFit, inverseM, leftCurve, rightCurve, vehicleOffset):
    plotY = np.linspace(0, undistortedFrame.shape[0]-1, undistortedFrame.shape[0])
    leftFitX = leftFit[0]*plotY**2 + leftFit[1]*plotY + leftFit[2]
    rightFitX = rightFit[0]*plotY**2 + rightFit[1]*plotY + rightFit[2]

    colorWarp = np.zeros((270, 512, 3), dtype='uint8')

    ptsLeft = np.array([np.transpose(np.vstack([leftFitX, plotY]))])
    ptsRight = np.array([np.flipud(np.transpose(np.vstack([rightFitX, plotY])))])
    pts = np.hstack((ptsLeft, ptsRight))

    cv2.fillPoly(colorWarp, np.int_([pts]), (150, 150, 150))

    unwarpedImage = cv2.warpPerspective(colorWarp, inverseM, (undistortedFrame.shape[1], undistortedFrame.shape[0]))
    result = cv2.addWeighted(undistortedFrame, 1, unwarpedImage, 0.3, 0)

    return result

def calculate_steering_angle(wheelbase, radius_of_curvature, inverse, max_steering_angle=25):
    if abs(radius_of_curvature) < 1e-6:
        raise ValueError("Radius of curvature is too small or zero, which is not realistic.")
    steering_angle_radians = math.atan(wheelbase / abs(radius_of_curvature))
    steering_angle_degrees = math.degrees(steering_angle_radians)
    if steering_angle_degrees == 0:
        return 0.0
    else:
        steering_angle_degrees = math.degrees(steering_angle_radians) + 5.0
    if radius_of_curvature < 0:
        steering_angle_degrees = -steering_angle_degrees - 5.0
    steering_angle_degrees = max(-max_steering_angle, min(max_steering_angle, steering_angle_degrees))
    steering_angle_degrees = round(steering_angle_degrees, 0) * 10.0
    if inverse:
        return -steering_angle_degrees
    return steering_angle_degrees