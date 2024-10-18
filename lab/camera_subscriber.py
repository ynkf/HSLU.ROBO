#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import time as time
import numpy as np

class CameraSubscriber:
    
    def __init__(self, robot_name):
        # create a openCV <-> ROS bridge
        self.cv2_bridge = CvBridge()
        
        self.image = None
        
        # subscribe to a topic of type CompressedImage  
        topic = '/' + robot_name + '/camera_node/image/compressed'
        # when a message is received, the callback is invoked
        rospy.Subscriber(topic, CompressedImage, self.callback)
        rospy.sleep(2.0) # needed to make sure the node is indeed initialized
        

    def callback(self, data):
        # the callback should be light and fast
        #rospy.loginfo("Received camera image of type: '%s'" % data.format)
        self.image = data
        
    def get_image_array(self):
        if self.image == None:
            return None
        return self.cv2_bridge.compressed_imgmsg_to_cv2(self.image, "bgr8")
