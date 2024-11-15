#!/usr/bin/env python3

import pyaudio
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData

# Initialize ROS node
rospy.init_node('ros_audio_to_virtual_mic')

# Retrieve parameters from ROS or use default values
FORMAT = pyaudio.paInt16  # 16-bit audio format
CHANNELS = rospy.get_param('~channels', 1)  # Mono by default
RATE = rospy.get_param('~rate', 16000)  # Sampling rate in Hz
CHUNK = rospy.get_param('~chunk_size', 1024)  # Buffer size
GAIN = rospy.get_param('~gain', 1.7)  # Gain factor for volume adjustment
DEVICE_KEYWORD = rospy.get_param('~device_keyword', 'Loopback')  # Device keyword to identify the microphone

# PyAudio initialization
p = pyaudio.PyAudio()

# Find the Loopback or virtual microphone device
device_index = None
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"Device {i}: {info['name']}")
    if DEVICE_KEYWORD in info['name']:  # Search for the device using the keyword
        device_index = i
        print(f"Found virtual device: Device {device_index} - {info['name']}")
        break

if device_index is None:
    raise RuntimeError("No virtual microphone found!")

# ROS callback for playing audio stream and adjusting volume
def audio_callback(msg):
    global audio_stream
    audio_data = np.frombuffer(msg.data, dtype=np.int16)

    # Adjust the volume using the gain factor
    audio_data = np.clip(audio_data * GAIN, -32768, 32767).astype(np.int16)

    # Write the adjusted audio data to the PyAudio stream
    audio_stream.write(audio_data.tobytes())

# Subscribe to the audio topic
rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, audio_callback)

# Open the PyAudio stream for output using the identified device
audio_stream = p.open(format=FORMAT,
                      channels=CHANNELS,
                      rate=RATE,
                      output=True,
                      output_device_index=device_index)

# Start the ROS event loop
rospy.spin()

# Clean up and close the PyAudio stream
audio_stream.stop_stream()
audio_stream.close()
p.terminate()
