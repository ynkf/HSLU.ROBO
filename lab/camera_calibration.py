#!/usr/bin/python3

from camera_subscriber import CameraSubscriber
from PIL import Image

if __name__ == "__main__":
    #wheels = WheelCommandPublisher("gamma")
    camera = CameraSubscriber("gamma")
    
    array = camera.get_image()
    img = Image.fromarray(array)
    img.save("./pic.bmp")
    