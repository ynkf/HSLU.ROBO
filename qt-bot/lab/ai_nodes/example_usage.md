# DetectronService

## Overview
Die `DetectronService` Node ist eine ROS-Node, die Objekterkennungsfunktionalität bietet. Sie abonniert das ROS-Topic `/camera/color/image_raw`, empfängt Bilder und wendet darauf Objekterkennung mit Hilfe von Detectron2 an. Anschließend gibt sie das annotierte Bild und die erkannten Bounding Boxes als Service-Antwort zurück.

## Usage
### Calling the Service from the Terminal
Um den `custom/cv/detectron/detect` Service aufzurufen, verwenden Sie den folgenden Befehl im Terminal:

```bash
rosservice call /custom/cv/detectron/detect
```

### Calling the Service Using Python
Sie können den Service auch in einem Python-Skript aufrufen. Hier ist ein Beispiel, wie Sie den Service aufrufen und die Antwort verarbeiten können:

```python
import rospy
from custom_interfaces.srv import Detectron
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Load the labels from the file 'labels.txt', make sure to include it.
labels = []
with open('./labels.txt', 'r') as f:
    for line in f:
        labels.append(line.strip())

rospy.init_node('detectron_client')
rospy.wait_for_service('custom/cv/detectron/detect')
try:
    detect_service = rospy.ServiceProxy('custom/cv/detectron/detect', Detectron)
    response = detect_service()

    # Verarbeitung der Antwort
    bridge = CvBridge()
    annotated_image = bridge.imgmsg_to_cv2(response.image, "bgr8")
    bounding_boxes = response.bounding_boxes

    for bbox in bounding_boxes:
        print(f"Bounding Box: xmin={bbox.xmin}, ymin={bbox.ymin}, xmax={bbox.xmax}, ymax={bbox.ymax}, class_id={bbox.class_id}")

except rospy.ServiceException as e:
    print(f"Service call failed: {e}")
```

## Response Data
Die Antwort des Detectron Service besteht aus folgenden Daten:

    image: Ein ROS Image-Nachrichtentyp (sensor_msgs/Image), der das annotierte Bild enthält.
    bounding_boxes: Eine Liste von BoundingBox Nachrichten, wobei jede BoundingBox folgende Felder hat:
        xmin: Die x-Koordinate des linken unteren Punktes der Bounding Box.
        ymin: Die y-Koordinate des linken unteren Punktes der Bounding Box.
        xmax: Die x-Koordinate des rechten oberen Punktes der Bounding Box.
        ymax: Die y-Koordinate des rechten oberen Punktes der Bounding Box.
        class_id: Die ID der erkannten Klasse des Objekts in der Bounding Box.


# SpeechRecognitionService

## Overview
Die `SpeechRecognitionService` Node ist eine ROS-Node, die Spracherkennungsfunktionalität bietet. Sie abonniert das ROS-Topic `/qt_respeaker_app/channel1`, um Audio-Daten zu empfangen, und bietet einen Service namens `/custom/speech/sr/recognize` zur Spracherkennung an. Die Node nutzt die `speech_recognition` Bibliothek, um die Sprache im Audio-Stream zu erkennen und den erkannten Text als Antwort zurückzugeben.

## Usage
### Calling the Service from the Terminal
Um den `/custom/speech/sr/recognize` Service aufzurufen, verwenden Sie den folgenden Befehl im Terminal:

```bash
rosservice call /custom/speech/sr/recognize "duration: 5
language: 'en-EN'"
```
`duration` gibt an, wie lange die Node auf Audio-Daten hören soll, und language ist der Sprachcode für die Spracherkennung (z.B. 'en-EN' für Englisch).

### Calling the Service Using Python
Sie können den Service auch in einem Python-Skript aufrufen. Hier ist ein Beispiel, wie Sie den Service aufrufen und die Antwort verarbeiten können:

```python
import rospy
from custom_interfaces.srv import SpeechRecognition

rospy.init_node('speech_recognition_client')
rospy.wait_for_service('/custom/speech/sr/recognize')
try:
    recognize_service = rospy.ServiceProxy('/custom/speech/sr/recognize', SpeechRecognition)
    response = recognize_service(duration=5, language='en-EN')

    recognized_text = response.text
    print(f"Recognized Text: {recognized_text}")

except rospy.ServiceException as e:
    print(f"Service call failed: {e}")
```

## Response Data
Die Antwort des SpeechRecognition Service besteht aus einem einzigen Feld:

    text: Ein String, der den von der Spracherkennung erkannten Text enthält.



