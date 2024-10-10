#!/usr/bin/python3

import cv2
import rospy
import numpy as np

from camera_subscriber import CameraSubscriber
from wheel_command_publisher import WheelCommandPublisher

class LineFollower:
    
    def __init__(self, robot_name):
        # initialize connection to physical components
        rospy.init_node('line_follower', anonymous=True)
        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        self.camera_subscriber = CameraSubscriber(robot_name)


    def run(self):
        masked = self.do_image_processing()

        # calculate a value for the left and the right sensory input
        half = int(masked.shape[1] / 2)
        left = masked[:,:half]
        right = masked[:,half:]

        black = [0, 0, 0]
        left_black = np.count_nonzero(np.all(left==black, axis=2))
        right_black = np.count_nonzero(np.all(right==black, axis=2))

        rospy.loginfo("black pixels on left side=" + str(left_black))
        rospy.loginfo("black pixels on right side=" + str(right_black))

        if (left_black < right_black):
            self.wheel_command_publisher.turn_wheels(0.1, 0.25)
        else:
            self.wheel_command_publisher.turn_wheels(0.25, 0.1)

    def stop(self):
        self.wheel_command_publisher.turn_wheels(0, 0)
        

    def do_image_processing(self):
        # image processing is done on the latest image received
        image = self.camera_subscriber.get_image()
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        yellow_lower_hsv = np.array([15, 100, 100])
        yellow_upper_hsv = np.array([40, 255, 255])
        mask = cv2.inRange(image_hsv, yellow_lower_hsv, yellow_upper_hsv)

        # show the part of the image that is within the color range
        return cv2.bitwise_and(image, image, mask=mask)



if __name__ == '__main__':
    lineFollower = LineFollower("alpha")
    x = 0
    while (x < 100):
        lineFollower.run()
        rospy.sleep(0.1)
        x += 1
    lineFollower.stop()

    
    