import numpy as np
import cv2

# Define the chess board rows and columns
rows = 5
cols = 9

criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 30, 0.001)

objectPoints = np.zeros((rows * cols, 3), np.float32)
objectPoints[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)

objectPointsArray = []
imgPointsArray = []

for i in range(1,21):
    print(str(i) + " ")
    img = cv2.imread("../camera_cal/calibration" + str(i) + ".jpg")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (rows, cols), None)

    # Make sure the chess board pattern was found in the image
    if ret:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objectPointsArray.append(objectPoints)
        imgPointsArray.append(corners)

        cv2.drawChessboardCorners(img, (rows, cols), corners, ret)

    cv2.imshow('chess board', img)
    cv2.waitKey(500)

# Calibrate the camera and save the results
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPointsArray, imgPointsArray, gray.shape[::-1], None, None)
np.savez('../camera_cal/calib.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)