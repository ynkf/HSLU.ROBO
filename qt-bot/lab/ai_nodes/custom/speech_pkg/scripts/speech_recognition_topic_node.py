#!/usr/bin/env python3

import rospy
from custom_interfaces.srv import TopicBasedSpeechRecognition, TopicBasedSpeechRecognitionResponse
import speech_recognition as sr
from audio_common_msgs.msg import AudioData
import io
import wave

class SpeechRecognitionTopicService:
    def __init__(self):
        rospy.init_node('topic_based_speech_recognition_service')
        self.recognizer = sr.Recognizer()
        # Initialize an empty BytesIO buffer for audio data
        self.audio_buffer = io.BytesIO()
        self.service = rospy.Service('/custom/speech/sr/topic_recognize', TopicBasedSpeechRecognition, self.recognize)

    def audio_callback(self, data):
        # Write raw audio data into the buffer
        self.audio_buffer.write(data.data)

    def recognize(self, req):
        # Subscribe to the audio topic and save data into the buffer
        audio_subscriber = rospy.Subscriber("/qt_respeaker_app/channel1", AudioData, self.audio_callback)
        rospy.sleep(req.duration)  # Wait for the specified duration to collect audio data
        audio_subscriber.unregister()  # Stop subscribing to the topic

        # Reset the buffer to the beginning
        self.audio_buffer.seek(0)

        # Convert the audio data in the buffer to WAV format
        with wave.open(self.audio_buffer, 'wb') as wf:
            wf.setnchannels(1)  # Mono audio
            wf.setsampwidth(2)  # 16-bit audio
            wf.setframerate(16000)  # Sample rate 16 kHz
            # Write the collected audio data into the WAV buffer
            wf.writeframes(self.audio_buffer.getvalue())

        # Reset the buffer to read it as an audio file
        self.audio_buffer.seek(0)

        # Use the audio file for speech recognition
        with sr.AudioFile(self.audio_buffer) as source:
            audio_data = self.recognizer.record(source)

        # Set the default language to English if no language is specified
        language = req.language if req.language else 'en-EN'

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

        # Prepare the buffer for the next usage
        self.audio_buffer = io.BytesIO()
        return TopicBasedSpeechRecognitionResponse(text)

if __name__ == '__main__':
    service = SpeechRecognitionTopicService()
    rospy.spin()
