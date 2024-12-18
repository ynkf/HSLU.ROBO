#!/usr/bin/env python3
import rospy
import time
import random

from std_msgs.msg import String

from qt_robot_interface.srv import behavior_talk_text
from qt_gesture_controller.srv import gesture_play
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition

from object_detection import objectDetection
from gemini_adapter import GeminiAdapter
from QTrobot_dev.qt_robot.src.players.challenger import Challenger
from player import Player

class ISpy():
    
    def __init__(self):
        rospy.wait_for_service('/qt_robot/speech/say')
        rospy.wait_for_service('/qt_robot/gesture/play')
        rospy.wait_for_service('/custom/speech/sr/microphone_recognize')
        rospy.loginfo('All services are available')
                
        self.talk_text_service = rospy.ServiceProxy('/qt_robot/speech/say', behavior_talk_text)
        self.gesture_play_service = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        self.object_detection = objectDetection()        
        self.speech_recognition_service = rospy.ServiceProxy('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition)
        rospy.loginfo('All services are initialized')
        
        self.object_whitelist = ['laptop', 'cell phone', 'bottle']
        
    
    def play_game(self, challenger: Challenger, player: Player):
        try:
            challenger.choose_spy_object()
            
            guessed_right = False
            while(not guessed_right):
                guessing_text = player.guess_object()
                
                challenger.check_spy_object(guessing_text)
                
            self.talk_text_service("You guessed the word!")           
            
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
            
            
        try:
            self.talk_text_service("Now you can choose an object and I try to guess it")
            
            self.gesture_play_service("QT/bye", 0)
            
            player_object = self.speech_recognition_service("en-US")
            
            objects = self.object_detection.detect_objects()
            
            object_list = [object for object in objects if object.class_name in self.object_whitelist]
            
            #TODO: guess an object in the list and check if its correct with ai service
            
            self.talk_text_service("ai response")
            
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
            
            
    def _choose_spy_object(self):
        objects = self.object_detection.detect_objects()
        object_list = [object for object in objects if object.class_name in self.object_whitelist]
            
        object_set = set(map(lambda x: x.class_name, object_list))
        return random.choice(list(object_set))
    
if __name__ == '__main__':
    rospy.init_node('i_spy_robot', anonymous=True)
    
    GeminiAdapter()
    i_spy = ISpy()
    
    # self.talk_text_service("Hello, let's play the game 'I spy'")
    # self.gesture_play_service("QT/bye", 0)
    i_spy.play_game()