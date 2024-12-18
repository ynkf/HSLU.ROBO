#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String

from qt_robot_interface.srv import emotion_show

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")
    
    rospy.wait_for_service('/qt_robot/emotion/show')
    emotion_show_service = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)


    
    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=5)
    rospy.sleep(2)
    # creating a ros publisher
    
    

    # publish a text message to TTS
    speechSay_pub.publish("Hello! my name is QT!")
    emotion_show_service("QT/showing_smile")
    
    speechSay_pub.publish("Hello! my name is Yannick!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")