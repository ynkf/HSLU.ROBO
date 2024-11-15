#!/usr/bin/env python3

import rospy
from custom_interfaces.srv import FaceDetection, FaceDetectionResponse
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
from deep_face import FaceDetector
import os
import cv2

class FaceDetectionService:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('face_detection_service')

        # Set the database path and initialize FaceDetector
        self.db_path = os.path.join(os.path.expanduser('~'), 'face_db')
        self.face_detector = FaceDetector(self.db_path)

        # Initialize CvBridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        self.image = None

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # Set up the face detection service
        self.detect_service = rospy.Service('custom/cv/deep_face/detect', FaceDetection, self.detect_faces)

        # Set up the reset database service using the ROS standard Trigger
        self.reset_service = rospy.Service('custom/cv/deep_face/reset', Trigger, self.reset_database)

    def image_callback(self, msg):
        """
        Callback function to receive image data from the topic and convert it to OpenCV format.
        """
        # Convert the ROS Image message to OpenCV format
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def detect_faces(self, request):
        """
        Service callback for face detection. Detects faces and returns the result as a JSON string.
        """
        if self.image is not None:
            # Save the current image temporarily
            temp_image_path = "temp_image.jpg"
            cv2.imwrite(temp_image_path, self.image)

            try:
                # Detect faces using FaceDetector
                recognized_faces = self.face_detector.detect_faces(temp_image_path)

                # Convert the detected faces dictionary to a JSON string
                faces_json = json.dumps(recognized_faces)

                # Create and return the service response with the JSON string
                return FaceDetectionResponse(json_result=faces_json)

            except Exception as e:
                rospy.logerr(f"Error in face detection: {e}")
                return FaceDetectionResponse(json_result="{}")

            finally:
                # Clean up the temporary image
                if os.path.exists(temp_image_path):
                    os.remove(temp_image_path)
        else:
            rospy.logerr("No image received for face detection.")
            return FaceDetectionResponse(json_result="{}")

    def reset_database(self, request):
        """
        Service callback to reset the face database.
        """
        try:
            # Call the reset_database method from the FaceDetector
            self.face_detector.reset_database()
            rospy.loginfo("Face database has been reset.")
            return TriggerResponse(success=True, message="Database reset successfully.")
        except Exception as e:
            rospy.logerr(f"Error resetting database: {e}")
            return TriggerResponse(success=False, message="Failed to reset database.")


if __name__ == '__main__':
    try:
        # Start the service node
        service = FaceDetectionService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
