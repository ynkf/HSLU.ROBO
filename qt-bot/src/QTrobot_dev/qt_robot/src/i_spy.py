#!/usr/bin/env python3
import rospy
import time
import random
import os


from std_msgs.msg import String
from std_msgs.msg import UInt8

from qt_robot_interface.srv import behavior_talk_text
from qt_gesture_controller.srv import gesture_play
from qt_robot_interface.srv import emotion_show
from qt_robot_interface.srv import audio_play
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition

from object_detection import objectDetection
from gemini_adapter import GeminiAdapter

class ISpy():
    
    def __init__(self):
        rospy.wait_for_service('/qt_robot/speech/say')
        rospy.wait_for_service('/qt_robot/gesture/play')
        rospy.wait_for_service('/qt_robot/emotion/show')
        rospy.wait_for_service('/custom/speech/sr/microphone_recognize')
        rospy.wait_for_service('/qt_robot/audio/play')
        rospy.loginfo('All services are available')
                
        self.talk_text_service = rospy.ServiceProxy('/qt_robot/speech/say', behavior_talk_text)
        self.gesture_play_service = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        self.emotion_show_service = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
        self.emotion_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=5)
        self.gesture_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=5)
        self.audio_service = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
        self.object_detection = objectDetection()        
        self.speech_recognition_service = rospy.ServiceProxy('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition)
        rospy.sleep(2)
        rospy.loginfo('All services are initialized')
        
        self.object_whitelist = ['laptop', 'cell phone', 'chair', 'bottle']
        
    
    def play_game(self):
        try:
            self.emotion_pub.publish("QT/showing_smile")
            self.talk_text_service("Hello, let's play the game 'I spy'")
                      
            spy_object = self._choose_spy_object()
            self.gesture_pub.publish("look_around_custom")
            rospy.sleep(2)

            self.gemini = GeminiAdapter(
                'We are playing a game of i spy i will shortly give you the object that currently is spied. '+
                'If I guess the object correct you will just return the text "correct". if its wrong you will just return the text "wrong".'+
                'Please answer all the questions about the spied object and if I ask for a hint give me a little hint.' +
                'the chosen spy object is ' + spy_object)

            i_spy_text = f'I spy something starting with the letter {spy_object[0]}'
            self.talk_text_service(i_spy_text)
            
            guessed_right = False
            while(not guessed_right):
                rospy.sleep(2)
                speech_input = self.speech_recognition_service("en-US")
                
                #TODO: check if speech_input.text predicts the correct object with ai service
                rospy.loginfo(f'User text to speech: {speech_input}')
                res = self.gemini.request(f'Users guess: {speech_input.text}')
                rospy.loginfo(res)
                if 'correct' in res.lower():
                    guessed_right = True
                    self.emotion_pub.publish("QT/happy")
                    self.talk_text_service("You guessed the word!")
                    self.gesture_play_service("QT/clapping", 0)
                else:
                    self.emotion_pub.publish("QT/confused")
                    self.talk_text_service(res)
            
            
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
            
            
        try:
            self.emotion_pub.publish("QT/showing_smile")
            self.talk_text_service("Now we switch roles")
                        
            objects = self.object_detection.detect_objects()
            
            object_list = [object for object in objects if object.class_name in self.object_whitelist]
            
            self.talk_text_service("Tell me the first letter of an object that you can see")
            
            player_object = self.speech_recognition_service("en-US")
            
            self.gemini = GeminiAdapter(
            'We are playing a game of i spy i will shortly give you the prompt to guess a specific item in the following list. '+
            'Item list: "' + str(object_list) + '".' + 'This is the player prompt: "' + player_object.text + '".'
            'Please guess the object and ask if it is the searched object.')
            
            response_list = []
            guessed_right = False
            while(not guessed_right):
                guessed_object = self.gemini.request("The already given answers: " + str(response_list) if response_list else "")
                self.talk_text_service(guessed_object)
                response_list.append(guessed_object)
                speech_input = self.speech_recognition_service("en-US")
                if "correct" in speech_input.text.lower():
                    guessed_right = True
                    self.emotion_pub.publish("QT/happy")
                    self.gesture_pub.publish("trauma_dance")
                    self.audio_service('Scooter.mp3', '')
                    
            
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
            
            
    def _choose_spy_object(self):
        objects = self.object_detection.detect_objects()
        object_list = [object for object in objects if object.class_name in self.object_whitelist]
            
        object_set = set(map(lambda x: x.class_name, object_list))
        return random.choice(list(object_set))
    
if __name__ == '__main__':
    rospy.init_node('i_spy_robot', anonymous=True)

    i_spy = ISpy() 
    i_spy.play_game()