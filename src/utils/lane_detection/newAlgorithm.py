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
    #cv2.imwrite("/home/oncst/slikePoslao/nasaSlika.jpg", image)
    # Step 1: Preprocessing
    gamma_corrected = adjust_gamma(image, gamma=3)  # Adjust brightness
    #cv2.imwrite("/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/gammaCorrected.jpg", gamma_corrected)
    blurred = cv2.GaussianBlur(gamma_corrected, (5, 5), 0)  # Apply low-pass filter
    #cv2.imwrite("/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/bluredImage.jpg", blurred)

    # Step 2: Convert to grayscale and apply Canny edge detection
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    #cv2.imwrite("/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/cannyImage.jpg", edges)

    # Step 3: Region of interest (ROI) to focus on the lane area
    height, width = edges.shape
    roi = np.zeros_like(edges)
    roi_vertices = np.array([[(20, 160), (width // 2 - 80, 20), (width // 2 + 80, 20), (width - 20, 160)]], dtype=np.int32)
    cv2.fillPoly(roi, roi_vertices, 255)
    masked_edges = cv2.bitwise_and(edges, roi)
    #cv2.imwrite("/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/maskedImages.jpg", masked_edges)


    # Step 4: Detect lane lines using Hough Transform
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi/180, threshold=30, minLineLength=40, maxLineGap=70)
    
    #output_image = np.zeros((height, width), dtype=np.uint8)

# Iterate over each detected line
    if lines is not None:
        #print(f"I got {len(lines)} lines")
        for line in lines:
            x1, y1, x2, y2 = line[0]  # Extract the start and end points of the line
            # Draw the line on the image (in color, for example green)
            #print(f"down:{x1},{y1}    up:{x2}, {y2}")
            #cv2.line(output_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
    #else:
        #print("I didnt get any lines whatsoever")


    # Optionally save the image with lines
    #cv2.imwrite('/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/houghTransform.jpg', output_image)

    #output_image2 = np.zeros((height, width), dtype = np.uint8)
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
                    #cv2.line(output_image2, (x1, y1), (x2, y2), (255, 255, 255), 2)
                elif -2 < slope < -0.5:  # Left lane
                    x1, y1, x2, y2 = line[0]
                    left_lines.append(line)
                    #cv2.line(output_image2, (x1, y1), (x2, y2), (255, 255, 255), 2)
    
    #cv2.imwrite('/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/leftRight.jpg', output_image2)
    #print(f"Number of left lanes is {len(left_lines)}")
    #print(f"Number of right lanes is {len(right_lines)}")

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
    
    #output_image3 = np.zeros((height, width), dtype = np.uint8)
    
    if left_lane is not None: 
        y_top = 0
        y_bottom = height
        x_top = y_top * left_lane[0] + left_lane[1]
        x_bottom = y_bottom * left_lane[0] + left_lane[1]
        #cv2.line(output_image3, (int(x_top), y_top), (int(x_bottom), y_bottom), (255, 255, 255), 2)
    if right_lane is not None:
        y_top = 0
        y_bottom = height
        x_top = y_top * right_lane[0] + right_lane[1]
        x_bottom = y_bottom * right_lane[0] + right_lane[1]
        #cv2.line(output_image3, (int(x_top), y_top), (int(x_bottom), y_bottom), (255, 255, 255), 2)
    
    #cv2.imwrite('/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/leftRight2.jpg', output_image3)

    #output_image = np.copy(image)
    
    # Compute lane center
    lane_center_x = width // 2
    left_x, right_x = None, None
    if left_lane is not None and right_lane is not None:
        y_bottom = height // 2  # Bottom of the image
        left_x = np.polyval(left_lane, y_bottom)
        right_x = np.polyval(right_lane, y_bottom)
        lane_center_x = int((left_x + right_x) / 2)
    elif left_lane is not None:
        return 22.0
    elif right_lane is not None:
        return -15.0
    else:
        return 0

    # Step 6: Calculate error (distance from the center of the image to the lane center)
    image_center_x = width // 2
    error = lane_center_x - image_center_x
    

    # Step 7: Use PID regulator to calculate the steering angle
    steering_angle = error / 80 * 25
    
    half_height = height // 2
    print(error)
    
    #steering_angle = math.atan(error / half_height)
    #steering_angle = max(-25, min(steering_angle, 25))

    # steering_angle = pid.compute(error, dt)

    #Step 8: Visualization
    
    #if left_lane is not None:
        #cv2.line(output_image, (int(np.polyval(left_lane, height)), height), (int(np.polyval(left_lane, height // 2)), height // 2), (255, 0, 0), 5)
    #if right_lane is not None:
        #cv2.line(output_image, (int(np.polyval(right_lane, height)), height), (int(np.polyval(right_lane, height // 2)), height // 2), (0, 0, 255), 5)
    #if left_x is not None and right_x is not None:
        # Draw lane center
        #cv2.line(output_image, (int(left_x), height), (int(right_x), height), (0, 255, 255), 2)  # Yellow line for lane width
        #cv2.circle(output_image, (lane_center_x, height), 5, (0, 255, 0), -1)  # Green circle for lane center
    #cv2.line(output_image, (image_center_x, height), (lane_center_x, height), (0, 255, 0), 2)
    #cv2.putText(output_image, f"Steering Angle: {steering_angle:.2f}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return steering_angle

# Main program
#if __name__ == "__main__":
 #   cap = cv2.imread("/mnt/c/Users/oncst/OneDrive/Desktop/noveSlike/slikePoslao/slike156.jpg", cv2.IMREAD_COLOR)  # Replace with your image path
  #  pid = PID(Kp=0.1, Ki=0.01, Kd=0.05)
   # dt = 1 / 30  # Assuming 30 FPS

    #output_frame, angle = lane_following(cap)
    #print(f"Steering Angle: {angle:.2f}")

    #cv2.imwrite("/mnt/c/Users/oncst/OneDrive/Desktop/outputImage/outputImage.jpg", output_frame)
    #qprint("Output image saved.")
