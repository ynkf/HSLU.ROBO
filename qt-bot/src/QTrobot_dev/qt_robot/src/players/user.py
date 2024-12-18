import rospy
import random

from custom_interfaces.srv import MicrophoneBasedSpeechRecognition

from object_detection import objectDetection
from QTrobot_dev.qt_robot.src.players.challenger import Challenger
from player import Player


class QtBot(Challenger, Player):
    
    def __init__(self):
        rospy.wait_for_service('/custom/speech/sr/microphone_recognize')
        
        self.speech_recognition_service = rospy.ServiceProxy('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition)
        self.spy_text = None

    
    def choose_spy_object(self) -> str:
        self.spy_text = self.speech_recognition_service("en-US")
        return self.spy_text
        
            
    def check_spy_object(self, guess: str):
        
        pass
    
    
    def guess_object(self):
        return self.speech_recognition_service("en-US")
