import cv2
import numpy as np

def increase_contrast(image, use_clahe=True):
    """
    Increases the contrast of an image.
    
    Args:
        image (numpy.ndarray): Input image (grayscale or color).
        use_clahe (bool): Whether to use CLAHE for better local contrast enhancement (default: True).
        
    Returns:
        numpy.ndarray: Image with increased contrast.
    """
    # If the image is grayscale
    if len(image.shape) == 2:
        if use_clahe:
            # Apply CLAHE for grayscale
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced_image = clahe.apply(image)
        else:
            # Apply global Histogram Equalization for grayscale
            enhanced_image = cv2.equalizeHist(image)
    
    # If the image is in color (BGR)
    else:
        # Convert to YCrCb color space
        ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
        y, cr, cb = cv2.split(ycrcb)
        
        if use_clahe:
            # Apply CLAHE to the Y (luminance) channel
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            y = clahe.apply(y)
        else:
            # Apply global Histogram Equalization to the Y channel
            y = cv2.equalizeHist(y)
        
        # Merge the channels back
        ycrcb = cv2.merge((y, cr, cb))
        # Convert back to BGR color space
        enhanced_image = cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
    
    cv2.imshow('high contrast', enhanced_image)
    return enhanced_image

def hsv(image):

    hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Mask for white color
    lower = np.array([0, 0, 200])
    upper = np.array([255, 30, 255])
    maskW = cv2.inRange(hsvImage, lower, upper)

    # Mask for yellow color
    lower = np.array([10, 100, 100])
    upper = np.array([30, 255, 255])
    maskY = cv2.inRange(hsvImage, lower, upper)

    mask = cv2.bitwise_xor(maskW, maskY)

    filteredImage = cv2.bitwise_and(image, image, mask = mask)

    return filteredImage

def canny(image, thr1, thr2):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 50, 150, cv2.THRESH_BINARY)
    canny = cv2.Canny(thresh, thr1, thr2)
    return canny

def warpImage(image):

    width = image.shape[1]
    height = image.shape[0]

    # Source points
    bottom_left = [round(width * 0.0), round(height * 0.9)]
    bottom_right = [round(width), round(height * 0.9)]
    top_left = [round(width * 0.3), round(height * 0.6)]
    top_right = [round(width * 0.7), round(height * 0.6)]

    src = np.float32([bottom_left, bottom_right, top_right, top_left])

    # Uncomment if want to show source points on the image
    # cv2.circle(image, bottom_left, 10, (0,0,255), -1)
    # cv2.circle(image, bottom_right, 10, (0,0,255), -1)
    # cv2.circle(image, top_left, 10, (0,0,255), -1)
    # cv2.circle(image, top_right, 10, (0,0,255), -1)
    # cv2.imshow("points", image)
    # cv2.waitKey(0)

    # Destination points
    bottom_left = [0, height]
    bottom_right = [width, height]
    top_left = [0, height * 0.25]
    top_right = [width, height * 0.25]

    dst = np.float32([bottom_left, bottom_right, top_right, top_left])

    M = cv2.getPerspectiveTransform(src, dst)
    inverseM = cv2.getPerspectiveTransform(dst, src)
    warpedImage = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_NEAREST)

    return warpedImage, inverseM
