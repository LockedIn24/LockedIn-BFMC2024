import cv2
import numpy as np
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, setpoint, current_value):
        umax = 5
        error = setpoint - current_value
        self.integral += self.Ki * error
        derivative = error - self.prev_error
        self.prev_error = error
        up = self.Kp * error
        if self.integral > umax:
            self.integral = umax
        elif self.integral < -umax:
            self.integral = -umax
        ud = self.Kd * derivative
        output = self.Kp * error + self.integral + self.Kd * derivative
        return output, up, self.integral, ud
    

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in range(256)]).astype("uint8")
    return cv2.LUT(image, table)

# Lane detection algorithm
def lane_following(image):
    # Step 1: Preprocessing
    gamma_corrected = adjust_gamma(image, gamma = 0.5)
    blurred = cv2.medianBlur(image, 5)  # Apply low-pass filter

    # Step 2: Convert to grayscale and apply Canny edge detection
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 300)

    # Step 3: Region of interest (ROI) to focus on the lane area
    height, width = edges.shape
    roi = np.zeros_like(edges)
    roi_vertices = np.array([[(70, 190), (400, 190), (300 , 50 ), (140, 50)]], dtype=np.int32)
    cv2.fillPoly(roi, roi_vertices, 255)
    masked_edges = cv2.bitwise_and(edges, roi)
    
    #Defining left and right roi
    roi_left = edges[220:250, 140:160]    
    roi_right = edges[ 220:250, 340:360]
    
    
    cv2.rectangle(image, (140, 220), (160, 250 ), (255,0,0), 2)
    cv2.rectangle(image, (340, 220), (360, 250), (255,0,0), 2)
    
    
    _, binary_left = cv2.threshold(roi_left, 230, 255, cv2.THRESH_BINARY)
    _, binary_right = cv2.threshold(roi_right, 230, 255, cv2.THRESH_BINARY)
    
    left_pixel_count = np.sum(binary_left == 255)
    right_pixel_count = np.sum(binary_right == 255)
    
    #cv2.putText(image, f"Left pixel count is: {left_pixel_count}", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #cv2.putText(image, f"Right pixel count is: {right_pixel_count}", (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    if right_pixel_count == 0:
        right_pixel_count = 1
    if left_pixel_count == 0:
        left_pixel_count = 1
        
    division_left = left_pixel_count / right_pixel_count
    division_right = right_pixel_count / left_pixel_count
    
    if division_left > 2 and left_pixel_count > 40:
        cv2.putText(image, "Mora da se skrene desnoooo", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return 25.0, image, True
    elif division_right > 2 and right_pixel_count > 40:
        cv2.putText(image, "Mora da se skrene levo", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return -25.0, image, True
 
    # Step 4: Detect lane lines using Hough Transform
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=80)
    

    # Step 5: Compute the lane center
    left_lines = []
    right_lines = []
    left_lane_slope = []
    right_lane_slope = []
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
                if 0.6 < slope < 2.5:  # Right lane
                    right_lane_slope.append(slope)
                    right_lines.append(line)
                elif -2.5 < slope < -0.6:  # Left lane
                    left_lane_slope.append(slope)
                    left_lines.append(line)
   
    
    average_right_slope = 0.0
    average_left_slope = 0.0
    
    if len(left_lane_slope) > 0:
        average_left_slope = sum(left_lane_slope) / len(left_lane_slope)
    if len(right_lane_slope) > 0: 
        average_right_slope = sum(right_lane_slope) / len(right_lane_slope)
        
    left_delete_counter = 0
    for i in range(0, len(left_lane_slope)):
        if abs(left_lane_slope[i] - average_left_slope) > 0.5:
            left_lines.pop(i - left_delete_counter)
            left_delete_counter += 1
            
    right_delete_counter = 0
    for i in range(0, len(right_lane_slope)):
        if abs(right_lane_slope[i] - average_right_slope) > 0.5:
            right_lines.pop(i - right_delete_counter)
            right_delete_counter += 1
        
   
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
    left_angle, right_angle, average_angle = None, None, None
    if left_lane is not None and right_lane is not None:
        left_lane_slope = np.polyder(left_lane)
        right_lane_slope = np.polyder(right_lane)
        
        if right_lane_slope + left_lane_slope > 0.2: 
            if abs(left_lane_slope) > right_lane_slope:
                return 22.0, image, False
            else:
                return -16.0, image, False

        y_bottom = height // 4 * 3  # Bottom of the image
        left_x = np.polyval(left_lane, y_bottom)
        right_x = np.polyval(right_lane, y_bottom)
        lane_center_x = int((left_x + right_x) / 2)
        left_angle = np.degrees(np.arctan(average_left_slope))
        right_angle = np.degrees(np.arctan(average_right_slope))
        average_angle = (left_angle + right_angle) / 2
    elif left_lane is not None:
        cv2.putText(image, f"Steering Angle: 22.0", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return 22.0, image, False
    elif right_lane is not None:
        cv2.putText(image, f"Steering Angle: -18.0", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return -19.0, image, False
    else:
        cv2.putText(image, f"Steering Angle: 0.0", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return 0, image, False

    # Step 6: Calculate error (distance from the center of the image to the lane center)
    image_center_x = width // 2 - 5
    error = lane_center_x - image_center_x

    #pid = PID(0.1, 0.01, 0.1)
    #steering_angle, up, ui, ud = pid.calculate(0, error)
    #print(f'{steering_angle} {up :.2f} {ui :.2f} {ud :.2f}')
    alpha = 1.0
    beta = 0.2
    
    steering_angle = alpha * np.degrees(np.arctan(error/(height / 4 * 3))) + beta * average_angle
    #steering_angle, up, ui, ud = pid.calculate(0, steering_angle)
    steering_angle = max(-25, min(25, steering_angle))
    #print(f'{steering_angle} {up :.2f} {ui :.2f} {ud :.2f}')

    # Step 8: Visualization
    
    if left_x is not None and right_x is not None:
        # Draw lane center
        cv2.line(image, (int(left_x), height), (int(right_x), height), (0, 255, 255), 2)  # Yellow line for lane width
        cv2.circle(image, (lane_center_x, height), 5, (0, 255, 0), -1)  # Green circle for lane center
    cv2.line(image, (image_center_x, height), (lane_center_x, height), (0, 255, 0), 2)
    cv2.putText(image, f"Steering Angle: {steering_angle:.2f}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return steering_angle, image, False
