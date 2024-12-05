#!/usr/bin/env python3
import rospy
import cv2
import time
import random

from std_msgs.msg import String
from sensor_msgs.msg import Image

from qt_robot_interface.srv import behavior_talk_text
from qt_gesture_controller.srv import gesture_play
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition
from custom_interfaces.srv import Detectron
from cv_bridge import CvBridge



def service_call():
    try:
        talk_text_service = rospy.ServiceProxy('/qt_robot/speech/say', behavior_talk_text)
        talk_text_service("Hello")
        
        gesture_play_service = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        gesture_play_service("QT/bye", 0)
        
        try:
            detect_service = rospy.ServiceProxy('custom/cv/detectron/detect', Detectron)
            response = detect_service()
            # Verarbeitung der Antwort
            bridge = CvBridge()
            
            annotated_image = bridge.imgmsg_to_cv2(response.image, "bgr8")
            cv2.imwrite("./src/images/object-detection-" + time.strftime("%H%M%S") + ".png", annotated_image)
            bounding_boxes = response.bounding_boxes
            
            object_whitelist = ['laptop', 'cell phone', 'bottle']
            object_list = [object for object in bounding_boxes if object.class_name in object_whitelist]

            for bbox in object_list:
                print(f"Bounding Box: xmin={bbox.xmin}, ymin={bbox.ymin}, xmax={bbox.xmax}, ymax={bbox.ymax}, class_id={bbox.class_id}, class_name={bbox.class_name}")
                
            object_set = set(map(lambda x: x.class_name, object_list))
            spy_object = random.choice(list(object_set))
            
            i_spy_text = f'I spy something starting with the letter {spy_object[0]}'
            talk_text_service(i_spy_text)
            

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        
        #speech_recognition_service = rospy.ServiceProxy('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition)
        #response = speech_recognition_service("en-US")

        #rospy.loginfo(response.text)

    except rospy.ServiceException as e:
        rospy.loginfo(f"Service call failed: {e}")
        

if __name__ == '__main__':
    rospy.init_node('my_package', anonymous=True)
    
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/gesture/play')
    rospy.wait_for_service('/custom/speech/sr/microphone_recognize')
    rospy.wait_for_service('custom/cv/detectron/detect')

    service_call()
