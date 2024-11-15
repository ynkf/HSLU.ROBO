#!/usr/bin/env python3

import rospy
from custom_interfaces.srv import Detectron, DetectronResponse
from custom_interfaces.msg import BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from detectron import ObjectDetector, LabelMapper



class DetectronService:
    def __init__(self):
        rospy.init_node('detectron_service')
        self.image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.capture_service = rospy.Service('custom/cv/detectron/detect', Detectron, self.detect)
        self.detectron = ObjectDetector()
        self.label_mapper = LabelMapper('/root/catkin_ws/src/custom/cv_pkg/scripts/labels.txt')

    def image_callback(self, msg):
        # Convert the ROS Image to OpenCV Image
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def detect(self, _):
        if self.image is not None:
            annotated_image, outputs = self.detectron.process_image(self.image)
            annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")

            # Erstelle eine Liste von BoundingBox-Nachrichten
            bounding_boxes_msgs = []
            for bb, cls in zip(outputs["instances"].pred_boxes.tensor, outputs["instances"].pred_classes):
                bbox_msg = BoundingBox()
                bbox_msg.xmin = bb[0].item()
                bbox_msg.ymin = bb[1].item()
                bbox_msg.xmax = bb[2].item()
                bbox_msg.ymax = bb[3].item()
                bbox_msg.class_id = cls.item()
                bbox_msg.class_name = self.label_mapper.get_class_name(cls.item())
                bounding_boxes_msgs.append(bbox_msg)

            # Erstelle und gebe die korrekte Service-Antwort zurück
            return DetectronResponse(image=annotated_image_msg, bounding_boxes=bounding_boxes_msgs)
        else:
            rospy.logerr("No image received.")
            # Beachten Sie, dass hier die leere Antwort richtig initialisiert werden muss
            return DetectronResponse()



if __name__ == '__main__':
    try:
        service = DetectronService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
