#!/usr/bin/python3

import cv2
import rospy
import numpy as np

from camera_subscriber import CameraSubscriber
from node_detection import NodeDetection
from wheel_command_publisher import WheelCommandPublisher

class LineFollower:
    
    def __init__(self, robot_name):
        # initialize connection to physical components
        rospy.init_node('line_follower', anonymous=True)
        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        self.camera_subscriber = CameraSubscriber(robot_name)


    def run(self):
        #finishes if on node
        #TODO: follow line until node detection
        is_node = False
        iteration = 1

        while(not is_node):
            image = self.camera_subscriber.get_image()

            self.follow_line(image, iteration)

    def follow_line(self, image, iteration):
        masked = self.do_image_processing(image, iteration)
        # cv2.imwrite("./images/masked-" + str(iteration) + ".jpg", masked)

        # calculate a value for the left and the right sensory input
        split = int(masked.shape[1] // 5)
        left = masked[:, :2 * split]
        middle = masked[:, 2 * split:3 * split]
        right = masked[:, 3 * split:]

        left_yellow = np.count_nonzero(left)
        middle_yellow = np.count_nonzero(middle)
        right_yellow = np.count_nonzero(right)

        rospy.loginfo("yellow pixels on left side=" + str(left_yellow))
        rospy.loginfo("yellow pixels on middle=" + str(middle_yellow))
        rospy.loginfo("yellow pixels on right side=" + str(right_yellow))


        if left_yellow < middle_yellow > right_yellow:
            self.wheel_command_publisher.turn_wheels(0.1, 0.1)
        elif left_yellow > right_yellow:
            self.wheel_command_publisher.turn_wheels(0.1, 0.35)
        else:
            self.wheel_command_publisher.turn_wheels(0.35, 0.1)

        return
    
    def stop(self):
        self.wheel_command_publisher.turn_wheels(0, 0)
        

    def do_image_processing(self, image, iteration):
        # image processing is done on the latest image received
        # image = self.camera_subscriber.get_image()
        cv2.imwrite("./images/image-" + str(iteration) + ".jpg", image)

        image = image[int(image.shape[0] // 2):image.shape[0] - 100]

        rospy.loginfo("image size: " + str(image.shape))

        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        yellow_lower_hsv = np.array([10, 100, 100])
        yellow_upper_hsv = np.array([55, 255, 255])
        mask = cv2.inRange(image_hsv, yellow_lower_hsv, yellow_upper_hsv)

        # show the part of the image that is within the color range
        return cv2.bitwise_and(image, image, mask=mask)



if __name__ == '__main__':
    lineFollower = LineFollower("alpha")
    x = 0
    while (x < 100):
        lineFollower.run(x)
        rospy.sleep(0.2)
        x += 1
    lineFollower.stop()