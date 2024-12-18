#!/usr/bin/env python3
import rospy
import cv2
import random
import time

from custom_interfaces.srv import Detectron
from cv_bridge import CvBridge

class objectDetection:
    
    def __init__(self):
        rospy.wait_for_service('custom/cv/detectron/detect')
        self.detect_service = rospy.ServiceProxy('custom/cv/detectron/detect', Detectron)
        
        
    def detect_objects(self):
        try:
            response = self.detect_service()
            bridge = CvBridge()
            
            annotated_image = bridge.imgmsg_to_cv2(response.image, "bgr8")
            cv2.imwrite("./src/images/object-detection-" + time.strftime("%H%M%S") + ".png", annotated_image)
            bounding_boxes = response.bounding_boxes

            # for bbox in bounding_boxes:
            #     print(f"Bounding Box: xmin={bbox.xmin}, ymin={bbox.ymin}, xmax={bbox.xmax}, ymax={bbox.ymax}, class_id={bbox.class_id}, class_name={bbox.class_name}")
            
            return bounding_boxes
            

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")