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

def calculateCurve(leftLaneIndicies, rightLaneIndicies, nonzeroX, nonzeroY):
    yEval = 719
    metersPerPixelY = 0.75/720  # meters per pixel in y dimension
    metersPerPixelX = 0.3/700  # meters per pixel in x dimension

    # Extract left and right line pixel positions
    leftX = nonzeroX[leftLaneIndicies]
    leftY = nonzeroY[leftLaneIndicies]
    rightX = nonzeroX[rightLaneIndicies]
    rightY = nonzeroY[rightLaneIndicies]

    if len(leftX) > 0 and len(leftY) > 0:
        leftFitCurve = np.polyfit(leftY * metersPerPixelY, leftX * metersPerPixelX, 2)
    else:
        leftFitCurve = [0, 0, 0]  # Default values if no left lane points are found

    if len(rightX) > 0 and len(rightY) > 0:
        rightFitCurve = np.polyfit(rightY * metersPerPixelY, rightX * metersPerPixelX, 2)
    else:
        rightFitCurve = [0, 0, 0]  # Default values if no right lane points are found

    # Calculate the radius of curvature
    leftCurveRadius = ((1 + (2 * leftFitCurve[0] * yEval * metersPerPixelY + leftFitCurve[1])**2)**1.5) / np.absolute(2 * leftFitCurve[0])
    rightCurveRadius = ((1 + (2 * rightFitCurve[0] * yEval * metersPerPixelY + rightFitCurve[1])**2)**1.5) / np.absolute(2 * rightFitCurve[0])

    return leftCurveRadius, rightCurveRadius

def showResult(undistortedFrame, leftFit, rightFit, inverseM, leftCurve, rightCurve, vehicleOffset):
    plotY = np.linspace(0, undistortedFrame.shape[0]-1, undistortedFrame.shape[0])
    leftFitX = leftFit[0]*plotY**2 + leftFit[1]*plotY + leftFit[2]
    rightFitX = rightFit[0]*plotY**2 + rightFit[1]*plotY + rightFit[2]

    colorWarp = np.zeros((720, 1280, 3), dtype='uint8')

    ptsLeft = np.array([np.transpose(np.vstack([leftFitX, plotY]))])
    ptsRight = np.array([np.flipud(np.transpose(np.vstack([rightFitX, plotY])))])
    pts = np.hstack((ptsLeft, ptsRight))

    cv2.fillPoly(colorWarp, np.int_([pts]), (150, 150, 150))

    unwarpedImage = cv2.warpPerspective(colorWarp, inverseM, (undistortedFrame.shape[1], undistortedFrame.shape[0]))
    result = cv2.addWeighted(undistortedFrame, 1, unwarpedImage, 0.3, 0)

    # Annotate lane curvature values and vehicle offset from center
    averageCurve = (leftCurve + rightCurve) / 2
    label = 'Left radius of curvature: %.1f m' % leftCurve
    result = cv2.putText(result, label, (30,40), 0, 1, (0,0,0), 2, cv2.LINE_AA)
    label = 'Right radius of curvature: %.1f m' % rightCurve

   # label = 'Vehicle offset from lane center: %.1f m' % vehicleOffset
    #result = cv2.putText(result, label, (30, 70), 0, 1, (0, 0, 0), 2, cv2.LINE_AA)
    return result
