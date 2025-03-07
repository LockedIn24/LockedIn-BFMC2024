import cv2
import numpy as np
import math

def calculateServoAngleTan(middle_of_lane, image_center, roi_height, max_angle = 25):
    deviation = middle_of_lane - image_center
    angle_rad = math.atan(deviation/roi_height)
    angle = math.degrees(angle_rad)
    angle = max(-max_angle, min(max_angle, angle))
    return angle

def calculateServoAngleOnc(middle_of_lane, image_center, height, max_angle = 25):
    error = middle_of_lane - image_center
    distance = height // 2
    wheelbaseLength = 360
    steering_angle_rad = math.atan(2*wheelbaseLength*error/distance**2)
    steering_angle = math.degrees(steering_angle_rad)
    steering_angle = max(-max_angle, min(steering_angle, max_angle))
    return steering_angle

# Gamma correction function
def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in range(256)]).astype("uint8")
    return cv2.LUT(image, table)

# PID Regulator Class
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        
        return output

# Lane detection algorithm
def lane_following(image):
    # Step 1: Preprocessing
    blurred = cv2.GaussianBlur(image, (5, 5), 0)  # Apply low-pass filter

    # Step 2: Convert to grayscale and apply Canny edge detection
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    canny_image = cv2.Canny(gray, 100, 300)

    # Step 3: Region of interest (ROI) to focus on the lane area
    height, width = canny_image.shape
    image_center = width // 2
    start_height = int(0.45 * height) 
    end_height = int(1 * height)
    start_width = int(0.1 * width) #dodato za processed video 1
    end_width = int(0.9 * width) #dodato za processed video 1

    mask = np.zeros(canny_image.shape[:2], dtype=np.uint8)
    pts = np.array([[start_width, start_height], [start_width, end_height], [end_width, start_height], [end_width, end_height]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.fillPoly(mask, [pts], 255)
    masked_image = cv2.bitwise_and(canny_image, canny_image, mask=mask)

    # Step 4: Detect lane lines using Hough Transform
    #lines = cv2.HoughLinesP(masked_image, rho=1, theta=np.pi/180, threshold=30, minLineLength=40, maxLineGap=70)
    lines = cv2.HoughLinesP(masked_image, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    # Step 5: Compute the lane center
    left_lane_x = []
    right_lane_x = []
    left_lane_y = []
    right_lane_y = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
            if slope > -2 and slope < -0.5:
                left_lane_x.extend([x1, x2])
                left_lane_y.extend([y1, y2])
                #cv2.line(masked_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            elif slope > 0.5 and slope < 2:
                right_lane_x.extend([x1, x2])
                right_lane_y.extend([y1, y2])
                #cv2.line(masked_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    left_lane_center = np.mean(left_lane_x) if left_lane_x else 0
    right_lane_center = np.mean(right_lane_x) if right_lane_x else width
    
    left_lane_x = np.array(left_lane_x)
    left_lane_y = np.array(left_lane_y)
    right_lane_x = np.array(right_lane_x)
    right_lane_y = np.array(right_lane_y)
    
    l_m, l_c = 0, 0
    r_m, r_c = 0, 0 
    if left_lane_x.size != 0:
        left_coefficients = np.polyfit(left_lane_x, left_lane_y, deg=1)
        l_m, l_c = left_coefficients  
    if right_lane_x.size != 0:
        right_coefficients = np.polyfit(right_lane_x, right_lane_y, deg=1)
        r_m, r_c = right_coefficients
    
    if l_m != 0:    
        x_min_left, x_max_left = min(left_lane_x), max(left_lane_x)  # Use detected x-points range
        y_min_left = int(l_m * x_min_left + l_c)
        y_max_left = int(l_m* x_max_left + l_c)
        cv2.line(image, (x_min_left, y_min_left), (x_max_left, y_max_left), (0, 255, 0), 2)  # Green line
    
    if r_m != 0:
        x_min_right, x_max_right = min(right_lane_x), max(right_lane_x)  # Use detected x-points range
        y_min_right = int(r_m * x_min_right + r_c)
        y_max_right = int(r_m* x_max_right + r_c)
        cv2.line(image, (x_min_right, y_min_right), (x_max_right, y_max_right), (0, 255, 0), 2)  # Green line
    
    
    
    middle_of_lane = (left_lane_center + right_lane_center) / 2

    # Step 6: Calculate angle
    servo_angle1 = calculateServoAngleTan(middle_of_lane, image_center, height//2)
    servo_angle2 = calculateServoAngleOnc(middle_of_lane, image_center, height)
         
    # Step 8: Visualization
    
    
    # Draw lane center

    # cv2.circle(image, (middle_of_lane, height), 5, (0, 255, 0), -1)  # Green circle for lane center
    # cv2.line(image, (width / 2, height), (middle_of_lane, height), (0, 255, 0), 2)
    # cv2.putText(image, f"Steering Angle: {servo_angle1:.2f}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return servo_angle1, image
