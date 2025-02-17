import cv2
import numpy as np
import math

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
    gamma_corrected = adjust_gamma(image, gamma=3)  # Adjust brightness
    blurred = cv2.GaussianBlur(gamma_corrected, (5, 5), 0)  # Apply low-pass filter

    # Step 2: Convert to grayscale and apply Canny edge detection
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)

    # Step 3: Region of interest (ROI) to focus on the lane area
    height, width = edges.shape
    roi = np.zeros_like(edges)
    roi_vertices = np.array([[(90, 250), (200, 90), (width - 200, 90), (width - 90, 250)]], dtype=np.int32)
    cv2.fillPoly(roi, roi_vertices, 255)
    masked_edges = cv2.bitwise_and(edges, roi)


    # Step 4: Detect lane lines using Hough Transform
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=30, minLineLength=40, maxLineGap=70)

    # Step 5: Compute the lane center
    left_lines = []
    right_lines = []
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
                if 0.5 < slope < 2:  # Right lane
                    right_lines.append(line)
                    x1, y1, x2, y2 = line[0]
                elif -2 < slope < -0.5:  # Left lane
                    x1, y1, x2, y2 = line[0]
                    left_lines.append(line)

    # Average left and right lanes
    def average_slope_intercept(lines):
        if len(lines) == 0:
            return None
        x_coords = []
        y_coords = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                x_coords.extend([x1, x2])
                y_coords.extend([y1, y2])
        poly = np.polyfit(y_coords, x_coords, 1)  # Fit a line: x = my + c
        return poly

    left_lane = average_slope_intercept(left_lines)
    right_lane = average_slope_intercept(right_lines) 

    # Compute lane center
    lane_center_x = width // 2
    if left_lane is not None:
        cv2.line(image, (int(np.polyval(left_lane, height)), height), (int(np.polyval(left_lane, height // 2)), height // 2), (255, 0, 0), 5)
    if right_lane is not None:
        cv2.line(image, (int(np.polyval(right_lane, height)), height), (int(np.polyval(right_lane, height // 2)), height // 2), (0, 0, 255), 5)
    
    left_x, right_x = None, None
    if left_lane is not None and right_lane is not None:
        y_bottom = height // 2  # Bottom of the image
        left_x = np.polyval(left_lane, y_bottom)
        right_x = np.polyval(right_lane, y_bottom)
        lane_center_x = int((left_x + right_x) / 2)
    elif left_lane is not None:
        cv2.putText(image, f"Steering Angle: 22.0", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return 22.0, image
    elif right_lane is not None:
        cv2.putText(image, f"Steering Angle: -15.0", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return -14.0, image
    else:
        cv2.putText(image, f"Steering Angle: 0.0", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return 0, image

    # Step 6: Calculate error (distance from the center of the image to the lane center)
    image_center_x = width // 2
    error = lane_center_x - image_center_x    

    
    
    distance = height // 2
    wheelbaseLength = 150
    steering_angle = math.atan(2*wheelbaseLength*error/distance**2)
    steering_angle = max(-25, min(steering_angle, 25))

    # Step 8: Visualization
    
    if left_x is not None and right_x is not None:
        # Draw lane center
        cv2.line(image, (int(left_x), height), (int(right_x), height), (0, 255, 255), 2)  # Yellow line for lane width
        cv2.circle(image, (lane_center_x, height), 5, (0, 255, 0), -1)  # Green circle for lane center
    cv2.line(image, (image_center_x, height), (lane_center_x, height), (0, 255, 0), 2)
    cv2.putText(image, f"Steering Angle: {steering_angle:.2f}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return steering_angle, image
