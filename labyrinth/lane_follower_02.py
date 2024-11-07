#!/usr/bin/python3

import cv2
import rospy
import numpy as np
import time

from camera_subscriber import CameraSubscriber
from wheel_command_publisher import WheelCommandPublisher

from node_detection import NodeDetection


class LaneFollower:
    
    def __init__(self, robot_name):
        # initialize connection to physical components
        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        self.camera_subscriber = CameraSubscriber(robot_name)
        self.node_detection = NodeDetection()
        self.avg_vel = 0.2
        
    def follow_line_until_node(self):
        #finishes if on node
        is_on_node = False
        iteration = 1

        while(not is_on_node):
            image = self.camera_subscriber.get_image()

            self.follow_line(image, iteration)
            is_on_node = self.node_detection.detect_on_node(image, iteration)
            rospy.sleep(0.2)
            iteration += 1

    def stop(self):
        self.wheel_command_publisher.turn_wheels(0, 0)

    def follow_line(self, image, iteration):
        image, region = self.do_image_processing(image, iteration)
        rospy.loginfo("iteration=" + str(iteration))
        cv2.imwrite("./labyrinth/images/test1/image-" + time.strftime("%H%M%S") + "-" + str(iteration) + ".jpg", image)

        # Distance resolution of the accumulator in pixels.
        rho = 1             
        # Angle resolution of the accumulator in radians.
        theta = np.pi/180   
        # Only lines that are greater than threshold will be returned.
        threshold = 20      
        # Line segments shorter than that are rejected.
        minLineLength = 20  
        # Maximum allowed gap between points on the same line to link them
        maxLineGap = 50

        # function returns an array containing dimensions of straight lines 
        # appearing in the input image
        hough = cv2.HoughLinesP(region, rho = rho, theta = theta, threshold = threshold, minLineLength = minLineLength, maxLineGap = maxLineGap)

        if hough is not None:
            for i in range(0, len(hough)):
                l = hough[i][0]
                cv2.line(image, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

        line = self.lane_lines(image, hough)
        if line == None:
            self.wheel_command_publisher.turn_wheels(0.1, 0.1)
            rospy.loginfo("lines were not detected correctly!!")
            return

        center_to_middle_line = ((image.shape[1] // 2, 480), line[1])
        rospy.loginfo("center_to_middle_line=" + str(center_to_middle_line))

        color=[255, 0, 0]
        thickness=12

        line_image = np.zeros_like(image)
        if line is not None:
            cv2.line(line_image, *line,  color, thickness)

        cv2.line(line_image, *center_to_middle_line,  [0, 255, 0], thickness)
        
        weighted_image = cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)

        cv2.imwrite("./labyrinth/images/test1/masked-" + time.strftime("%H%M%S") + "-" + str(iteration) + ".jpg", weighted_image)

        center_middle_slope = (center_to_middle_line[0][0] - center_to_middle_line[1][0]) / (center_to_middle_line[0][1] - center_to_middle_line[1][1])

        rospy.loginfo("middle_slope=" + str(center_middle_slope))

        left_vel = self.avg_vel - (center_middle_slope * 0.3)
        if left_vel < 0.1:
            left_vel = 0.1
        elif left_vel > 0.4:
            left_vel = 0.4
        rospy.loginfo("left_vel=" + str(left_vel))
   
        right_vel = self.avg_vel + (center_middle_slope * 0.3)
        if right_vel < 0.1:
            right_vel = 0.1
        elif right_vel > 0.4:
            right_vel = 0.4
        rospy.loginfo("rigth_vel=" + str(right_vel))

        self.wheel_command_publisher.turn_wheels(left_vel, right_vel)

    def stop(self):
        self.wheel_command_publisher.turn_wheels(0, 0)
        

    def do_image_processing(self, image, iteration):
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        kernel_size = 5
        # Applying gaussian blur to remove noise from the frames
        blur = cv2.GaussianBlur(grayscale, (kernel_size, kernel_size), 0)

        low_t = 50
        # second threshold for the hysteresis procedure 
        high_t = 150
        # applying canny edge detection and save edges in a variable
        edges = cv2.Canny(blur, low_t, high_t)

        # create an array of the same size as of the input image 
        mask = np.zeros_like(edges)   
        # if you pass an image with more then one channel
        if len(edges.shape) > 2:
            channel_count = edges.shape[2]
            ignore_mask_color = (255,) * channel_count
        # our image only has one channel so it will go under "else"
        else:
            # color of the mask polygon (white)
            ignore_mask_color = 255
        # creating a polygon to focus only on the road in the picture
        # we have created this polygon in accordance to how the camera was placed
        rows, cols = edges.shape[:2]
        bottom_left  = [cols * 0.2, rows * 0.85]
        top_left     = [cols * 0.3, rows * 0.6]
        bottom_right = [cols * 0.8, rows * 0.85]
        top_right    = [cols * 0.7, rows * 0.6]
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        # filling the polygon with white color and generating the final mask
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        # performing Bitwise AND on the input image and mask to get only the edges on the road
        region = cv2.bitwise_and(edges, mask)

        # show the part of the image that is within the color range
        return image, region

    def lane_lines(self, image, lines):
        """
        Create full lenght lines from pixel points.
            Parameters:
                image: The input test image.
                lines: The output lines from Hough Transform.
        """
        line = self.average_slope_intercept(image, lines)

        slope, intercept = (0, 0)
        try:
            slope, intercept = line
        except TypeError:
            return None

        y1 = image.shape[0]
        y2 = y1 * 0.6
        line  = self.pixel_points(y1, y2, line)

        return line


    def average_slope_intercept(self, image, lines):
        # TODO: what if only one side of the lane was detected?
        """
        Find the slope and intercept of the left and right lanes of each image.
        Parameters:
            lines: output from Hough Transform
        """
        line_array    = [] #(slope, intercept)
        weights  = [] #(length,)
        
        rospy.loginfo("print all hough lines")
        
        for line in lines:
            rospy.loginfo(line)
            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue
                # calculating slope of a line
                slope = (y2 - y1) / (x2 - x1)
                # calculating intercept of a line
                intercept = y1 - (slope * x1)
                # calculating length of a line
                length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
                
                # TODO: should we reduce slope range??
                if slope > -1 and slope < 1: 
                    line_array.append((slope, intercept))
                    weights.append((length))
                
        line  = np.dot(weights,  line_array) / np.sum(weights)  if len(weights) > 0 else None
        return line
    

    def pixel_points(self, y1, y2, line):
        """
        Converts the slope and intercept of each line into pixel points.
            Parameters:
                y1: y-value of the line's starting point.
                y2: y-value of the line's end point.
                line: The slope and intercept of the line.
        """
        if line is None:
            return None
        slope, intercept = line
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))


if __name__ == '__main__':
    laneFollower = LaneFollower("alpha")
    x = 0
    while (x < 100):
        laneFollower.run(x)
        rospy.sleep(0.2)
        x += 1
    laneFollower.stop()
