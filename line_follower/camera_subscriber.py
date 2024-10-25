#!/usr/bin/python3

import time
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class CameraSubscriber:
    
    def __init__(self, robot_name):
        # # initialize a node with a name, annonymous=True ensures that the name is unique
        # rospy.init_node('camera_listener', anonymous=True)
        # subscribe to a topic of type CompressedImage  
        topic = '/' + robot_name + '/camera_node/image/compressed'
        # when a message is received, the callback is invoked
        rospy.Subscriber(topic, CompressedImage, self.callback)
        rospy.sleep(2.0) # needed to make sure the node is indeed initialized
        
        # create a openCV <-> ROS bridge
        self.cv2_bridge = CvBridge()
        self.rate = rospy.Rate(10)  # the node is running at 10 hz

    def callback(self, data):
        # the callback should be light and fast
        rospy.loginfo("Received camera image of type: '%s'" % data.format)
        self.image = data

    def get_image(self):
        # image processing is done on the latest image received
        return self.cv2_bridge.compressed_imgmsg_to_cv2(self.image, "bgr8")

        
    def run(self):
        self.do_image_processing()
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('camera_listener', anonymous=True)
    robot_name = "alpha"
    cs = CameraSubscriber(robot_name)
    image = cs.get_image()
    cv2.imwrite("./images/labyrinth/image-" + time.strftime("%H%M%S") + ".jpg", image)
