import cv2
import numpy as np

class NodeDetection:
    
    def __init__(self, robot_name):
        rospy.init_node('NodeDetection', anonymous=True)
        self.previous_red_seen = False
        self.red_seen = False
        self.node_index = 0
        
    
    def do_image_processing(self, image):
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
        w_split = w // 3
        h_split = h // 6
        masked_w_split = masked[:, w_split:2 * w_split]
        node_image = masked_w_split[5 * h_split:, :]
        
        return node_image
    
    
    def get_node_index(self, image):
        # Check for nodes (for every image)
        red_pixel_count = np.count_nonzero(self.do_image_processing(image))
        if red_pixel_count > 100:
            previous_red_seen = red_seen
            red_seen = True
        elif red_pixel_count <= 100:
            previous_red_seen = red_seen
            red_seen = False  
            if previous_red_seen == True and red_seen == False:
                self.node_index += 1
                return self.node_index
        return None