#!/usr/bin/python3
import os
import rospy
import cv2 as cv
import numpy as np
import math

from wheel_command_publisher import WheelCommandPublisher
from camera_subscriber import CameraSubscriber

BASE_SPEED = 0.4

class LineFollower:
    
    def __init__(self, robot_name):
        self.wcp = WheelCommandPublisher(robot_name)
        self.cs = CameraSubscriber(robot_name)
        
    def do_update(self):
        img = self.cs.get_image_array()
        if img is None:
            rospy.loginfo("No image found. Skipping...")
            return
        
        # IMAGE PROCESSING
        # ================
        SCALE = (100, 75)
        scaled = cv.resize(img, SCALE)
        height, width = scaled.shape

        CROP_AMOUNT = 0.5
        cropped = scaled[int(height * CROP_AMOUNT):height, :]
        height, width = cropped.shape
        
        gray = cv.cvtColor(cropped, cv.COLOR_BGR2GRAY)

        thresh = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 15, 1)
        
        contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        # SET STEERING
        # ============
        turning = 0

        border_x = self._find_boarder_x(contours, gray)
        if border_x is not None:
            # Turn left to avoid the border
            turning += (border_x - (width-1)) / (width-1)
            
        center_x = self._find_center_line_x(contours, cropped)
        if center_x is not None:
            # Turn right to avoid center line
            turning += center_x / (width-1)
            
        rospy.loginfo(f"Centerline: {center_x} | Border (from right side of image): {border_x - (width-1)}")
            
        # STEER
        # =====
        turning *= BASE_SPEED
        left = BASE_SPEED + min(turning, 0)
        right = BASE_SPEED - max(turning, 0)
        rospy.loginfo(f"Deviation: {turning}, Left wheel: {left}, Right wheel: {right}")
        
        self.wcp.set_left_right(left, right)
            
    def _find_boarder_x(self, contours, image):
        x = None
        if len(contours) > 0:
            max_score = -1
            # Find the brightest contour which is our white line
            for contour in contours:
                # Create mask
                mask = np.zeros_like(image, dtype=np.uint8)
                mask = cv.drawContours(mask, [contour], -1, (255), thickness=cv.FILLED)
                # Calculate the mean brightness using the mask
                mean_val = np.mean(cv.mean(image, mask=mask)) / 255  # Only take the first value since the image is grayscale
                if mean_val < 0.8:
                    # Ignore non bright contours
                    continue
                
                m = cv.moments(contour)
                area = m['m00']
                if area == 0:
                    continue
                score = math.log10(area) * math.pow(mean_val / 255, 2)
                
                if score > max_score:
                    max_score = score
                    cx = int(m['m10'] / m['m00'])  # X coordinate of contours centroid
                    x = cx
        return x

    def _find_center_line_x(self, contours, image):
        if len(contours) == 0:
            return None
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        segments = []
        for contour in contours:
            # Create mask
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            mask = cv.drawContours(mask, [contour], -1, 255, thickness=cv.FILLED)
            # Calculate the yellowennes
            hue = cv.mean(hsv[:, :, 0], mask=mask)[0]
            saturation = cv.mean(hsv[:, :, 1], mask=mask)[0]
            if saturation < 100 or hue < 10 or hue > 40:
                # Ignore non yellow contours
                continue
            
            m = cv.moments(contour)
            if m['m00'] == 0:
                continue
            cx = int(m['m10'] / m['m00'])  # X coordinate of contours centroid
            segments.append(cx)
        
        if len(segments) == 0:
            return None
        return int(np.mean(segments))
        
        
    def run(self, rate=10):
        rospy.loginfo(f"Starting Linefollower loop")
        rate = rospy.Rate(rate)
        try:
            while not rospy.is_shutdown():
                self.do_update()
                rate.sleep()
        finally:
            self.wcp.set_left_right(0, 0)

if __name__ == "__main__":
    robot_name = os.environ["VEHICLE_NAME"]
    rospy.loginfo(f"Starting up Linefollower for robot: {robot_name}")
    rospy.init_node("line_follower", anonymous=False)

    follower = LineFollower(robot_name)
    rospy.loginfo(f"Linefollower initialized")
    follower.run()
    