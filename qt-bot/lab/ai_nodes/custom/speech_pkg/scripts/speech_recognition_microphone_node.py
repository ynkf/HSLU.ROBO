#!/usr/bin/env python3 

import rospy
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition, MicrophoneBasedSpeechRecognitionResponse
import speech_recognition as sr

class SpeechRecognitionMicrophoneService:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('microphone_based_speech_recognition_service')
        
        # Create a speech recognizer instance
        self.recognizer = sr.Recognizer()

        # Get microphone device index, sample rate, and timeout from ROS parameters or set defaults
        mic_index = rospy.get_param('~microphone_device_index', 1)
        sample_rate = rospy.get_param('~sample_rate', 16000)  # Default to 16000 Hz
        self.timeout = rospy.get_param('~timeout', 10)  # Default timeout of 10 seconds

        # Create a microphone instance using the selected device index and sample rate
        self.microphone = sr.Microphone(device_index=mic_index, sample_rate=sample_rate)
        
        # Set the energy threshold for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        
        # Initialize the ROS service
        self.service = rospy.Service('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition, self.recognize)

    def recognize(self, req):
        # Set the default language to English if no language is specified
        language = req.language if req.language else 'en-EN'

        # Perform the speech recognition
        with self.microphone as source:
            rospy.loginfo("Please speak now")
            audio_data = self.recognizer.listen(source, timeout=self.timeout)
        
        # Attempt speech recognition
        try:
            text = self.recognizer.recognize_google(audio_data, language=language)
            rospy.loginfo(f"Recognized text: {text}")
        except sr.UnknownValueError:
            text = ''
            rospy.logwarn("Speech recognition could not understand audio")
        except sr.RequestError as e:
            text = ''
            rospy.logwarn(f"Could not request results from speech recognition service; {e}")
        
        return MicrophoneBasedSpeechRecognitionResponse(text)

if __name__ == '__main__':
    service = SpeechRecognitionMicrophoneService()
    rospy.spin()
