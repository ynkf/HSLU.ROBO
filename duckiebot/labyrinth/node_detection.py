#!/usr/bin/python3

import cv2
import numpy as np
import rospy

class NodeDetection:
    
    def __init__(self):
        self.previous_red_seen = False
        self.red_seen = False
        self.node_index = 0
        
    
    def do_image_processing(self, image, iteration):
        # Color image to HSV        
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define HSV range
        red_lower_hsv_1 = np.array([0, 110, 100])
        red_upper_hsv_1 = np.array([10, 255, 255])

        red_lower_hsv_2 = np.array([160, 110, 100])
        red_upper_hsv_2 = np.array([180, 255, 255])
    
        # Create new mask
        mask_1 = cv2.inRange(image_hsv, red_lower_hsv_1, red_upper_hsv_1)
        mask_2 = cv2.inRange(image_hsv, red_lower_hsv_2, red_upper_hsv_2)
        
        # Combine the two masks using a bitwise OR
        mask = cv2.bitwise_or(mask_1, mask_2)
        
        # Show the part of the image that is within the color range
        masked = cv2.bitwise_and(image, image, mask=mask)

        # Define variables for picture shape
        h, w, channels = masked.shape

        # Split the image vertically
        w_split = w // 5
        h_split = h // 6
        masked_w_split = masked[:, 2 * w_split:4 * w_split]
        node_image = masked_w_split[5 * h_split:, :]

        cv2.imwrite("./images/nd-mask-" + str(iteration) + ".jpg", node_image)
        return node_image
    
    
    def detect_on_node(self, image, iteration):
        # Check for nodes (for every image)
        # TODO: this does not work
        red_pixel_count = np.count_nonzero(self.do_image_processing(image, iteration))
        if red_pixel_count > 100:
            previous_red_seen = self.red_seen
            self.red_seen = True
        elif red_pixel_count <= 50:
            previous_red_seen = self.red_seen
            self.red_seen = False  
            if previous_red_seen == True and self.red_seen == False:
                self.node_index += 1
                return True
        return False