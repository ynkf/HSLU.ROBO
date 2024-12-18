#!/usr/bin/env python3
import rospy

from std_msgs.msg import String

from qt_robot_interface.srv import behavior_talk_text
from qt_gesture_controller.srv import gesture_play
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition

def service_call():
    try:
        talk_text_service = rospy.ServiceProxy('/qt_robot/speech/say', behavior_talk_text)
        talk_text_service("Hi Yannick")
        
        gesture_play_service = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        gesture_play_service("QT/bye", 0)
        
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

    service_call()