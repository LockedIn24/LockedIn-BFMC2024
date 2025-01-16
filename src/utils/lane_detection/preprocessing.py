import cv2
import numpy as np

def hsv(image):

    hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Mask for white color
    lower = np.array([0, 0, 200])
    upper = np.array([255, 30, 255])
    maskW = cv2.inRange(hsvImage, lower, upper)

    filteredImage = cv2.bitwise_and(image, image, mask = maskW)

    return filteredImage

def canny(image, thr1, thr2):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray', gray)
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 50, 240, cv2.THRESH_BINARY)
    # cv2.imshow('thresh', thresh)
    canny = cv2.Canny(thresh, thr1, thr2)
    return canny

def warpImage(image):

    width = image.shape[1]
    height = image.shape[0]
    
    # Source points
    bottom_left = [round(0), round(height * 0.8)]
    bottom_right = [round(width), round(height * 0.8)]
    top_left = [round(width * 0.3), round(height * 0.45)]
    top_right = [round(width * 0.7), round(height * 0.45)]
    
    src = np.float32([bottom_left, bottom_right, top_right, top_left])

    # Destination points
    bottom_left = [round(width * 0.2), height]
    bottom_right = [round(width * 0.8), height]
    top_left = [round(width * 0.3), 0]
    top_right = [round(width * 0.7), 0]
    
    dst = np.float32([bottom_left, bottom_right, top_right, top_left])

    M = cv2.getPerspectiveTransform(src, dst)
    inverseM = cv2.getPerspectiveTransform(dst, src)
    warpedImage = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_NEAREST)
    cv2.imshow('warped', warpedImage)

    return warpedImage, inverseM