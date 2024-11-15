# Arbeitsumgebung

## Voraussetzungen
* VSCode with Docker and Dev Containers extensions


## Roboter
Für den Unterricht stehen euch drei Roboter zur Verfügung. Zu jedem QtRobot gehört ein Mini-PC (NUC):

|QtRobot | NUC | IP des NUC |
|-|-|-|
| QTRD000339 | robo-nuc-qt1 |`192.168.1.201`
| QTRD000353 | robo-nuc-qt2 |`192.168.1.202`
| QTRD000375 | robo-nuc-qt3 |`192.168.1.203`


Die wichtigsten Aktoren und Sensoren sind [hier](QTRobot.md) beschrieben.

## Quick Start
1. Verbinde dich mit dem selben Netzwerk wie der QtRobot 

       SSID: robo-qt, PW: uncannyvalley

1. Klone dieses Repository und öffne es in VSCode.

1. In VSCode, navigiere zu *File* > *Preferences* > *Settings* > *Docker Environment* und füge folgenden Eintrag hinzu: `DOCKER_HOST` mit `ssh://robo@192.168.1.201`, bzw der IP Adresse, die zu eurem NUC gehört (siehe Tabelle oben). Das Vorgehen ist jeh nach Betriebssystem etwas anders, daher nachfolgend eine Betriebssystem spezifische Anleitung:

        Windows: In VS Code, navigate to File > Preferences > Settings > Docker Environment.

        Mac: In VS Code, click on the Manage icon (located in the lower-left corner) > Settings > Docker > Docker: Environment.

1. Erstelle ein ssh-key-pair und kopiere deinen public key auf den entsprechenden NUC.

       ssh-copy-id robo@192.168.1.201

   Das Passwort für die NUCs lautet `uncannyvalley`.

1. Erstelle ein Docker Kontext, der auf den jweiligen Intel Nuc zeigt:
    `docker context create qt-robot --docker "host=ssh://robo@192.168.1.201:22"`
`
1. Benutze die Command Palette (Ctrl+Shift+P) um den Kontext zu aktivieren: Use command to activate the Docker context.

1. Starte den Docker Container, indem du in VSCode mit einem Rechtsklick auf `docker-compose.yml` klickst und `Compose Up` auswählst.

1. Klicke in der linken unteren Ecke auf das Pfeil-Symbol und wähle `Attach to Running Container` aus.


1. Wenn die vorangehenden Schritte erfolgreich waren, sollte sich nun ein weiteres VSCode Fenster öffnen, welches mit dem Docker Container auf dem Intel NUC verbunden ist.
In dieser Umgebung kann nun entwickelt werden. Der Docker Container hat bereits ROS und alle benötigten Pakete installiert.

1. Weiterhin ist es ratsam eine eigene Versionskontrolle, wie Git zu verwenden.

1. Um zu testen, ob alles funktioniert, ist es ratsam die Befehle aus dem Dokument [introduction_ROS.ipynb](introduction_ROS.ipynb) im Terminal auszuführen.

1. Es ist möglich den Container mit Compose Down zu stoppen, jedoch wird dann der Inhalt (z.B. installierte Pakete) gelöscht.
Es ist daher ratsam, in VSCode auf den Reiter "Docker" zu wechseln und dort den Container zu stoppen.
Wenn die Entwicklung weiter gehen soll, kann der Container ebenfalls in diesem Reiter gestartet werden. 
So bleiben alle installierten Pakete erhalten.

## Entwicklung
1. Wechseln in den Ordner `/root/catkin_ws/src/QTrobot_dev` im Docker Container
    ```bash
    cd /root/catkin_ws/src/QTrobot_dev
    ```

2. Zu beachten ist, dass nur der Ordner `~/root/catkin_ws/src/QTrobot_dev` "abgesichert ist". Entsprechend ist es ratsam, in diesem Ordner zu entwickeln. Wenn ihr den Container dann aus Versehen zerstört, ist euer Fortschritt nicht verloren. Alles Ausserhalb dieses Ordners würde in diesem Fall zurückgesetzt. Der Ordner `/root/catkin_ws/src/QTrobot_example` beinhaltet eine Beispiel ROS-Node. Kopiere einfach den inhalt in `/root/catkin_ws/src/QTrobot_dev`, um einen Startpunkt zu haben.

3. Alternativ kannst du auch mit folgendem Befehl ein eigenes ROS Paket erstellen:

       catkin_create_pkg my_package std_msgs rospy roscpp -D "My ros package" -V "1.0.0" -a "my name"

   Eine ausführbare Python Datei kannst du folgendermassen erstellen:
  
    ```bash
    cd my_package/src
    touch my_ros_node.py
    chmod +x my_ros_node.py
    ```
   
   Das neu erstellte Python Script kann nun in VSCode geöffnet werden und die Entwicklung kann beginnen.
Weitere Hilfestellungungen zur Entwicklung findet ihr [auf der Seite des Herstellers](https://docs.luxai.com/docs/tutorials/intro_ros).

## Custom AI - Nodes
Einige Implementationen vom QTRobot sind nicht wirklich zufriedenstellend. Dazu gehört die Objekt-, die Personen- und die Spracherkennung.

WICHTIG: Diese Nodes laufen bereits auf dem QTRobot und sollen daher NICHT lokal oder auf dem Intel NUC gestartet werden. Der Ordner `ai_nodes` in diesem Repo kann daher ignoriert werden.

Wir haben uns daher dazu entschieden, für diese Dienste eine alternative Implementation zu machen. daraus sind folgende vier ROS-Services entstanden:

Die Nachfolgenden Unterkaptizel zeigen, wie die Services verwendet werden können.

### Face Detection
Die face detection Node bietet eine Gesichtserkennung. Sie abonniert das ROS-Topic `/camera/color/image_raw`, empfängt Bilder und wendet darauf Gesichtserkennung mit DeepFace an. Anschliessend werden für erkannte Gesichter eindeutige ID's und einige weitere Informationen zurückgegeben. Weiter gibt es einen Service, um die Datenbank der Gesichter zurückzusetzen.

Um den `/custom/cv/deep_face/detect` Service aufzurufen, verwenden Sie den folgenden Befehl im Terminal:

```bash
rosservice call /custom/cv/deep_face/detect "{}"
```

Um die Datenbank zurückzusetzen, kann folgender Befehl verwendet werden:

```bash
rosservice call /custom/cv/deep_face/reset "{}"
```

Die Antwort des face detection Service ist relativ komplex. Daher wird ein JSON-Dump zurückgegeben. D. h. der Return ses Service ist ein String, der anschliessend mit Python wieder in ein dictionary umgewandelt werden kann. Das dict enthällt viele Informationen zu den erkannten Personen, wie beispielsweise Alter, Geschlecht, Ethnie, Emotionen etc.

### Speech Recognition

die speech to text Node bietet eine verbesserte Erkennung von Sprache. Sie basiert auf dem Topic `/qt_respeaker_app/channel0`. Dieses Topic gibt die Aufname eines Mikrofons an die Node weiter. Das ermöglicht die Erkennung verschiedener Sprachen.

Um den `/custom/speech/sr/microphone_recognize` Service aufzurufen, verwenden Sie den folgenden Befehl im Terminal:

```bash
rosservice call /custom/speech/sr/microphone_recognize "language: 'en-US'" 
```
Die zu erwartende Sprache kann einfach angepasst werden: en-US, de-DE, dr-FR



Der Service gibt einen String zurück. Dabei handelt es sich um den erkannten Text.

text: erkannte Spracheingabe

### Object detection
Die object detection Node ist eine ROS-Node, die Objekterkennungsfunktionalität bietet. Sie abonniert das ROS-Topic `/camera/color/image_raw`, empfängt Bilder und wendet darauf Objekterkennung mit Hilfe von Detectron2 an. Anschließend gibt sie das annotierte Bild und die erkannten Bounding Boxes mit Labels als Service-Antwort zurück.

Um den `custom/cv/detectron/detect` Service aufzurufen, verwenden Sie den folgenden Befehl im Terminal:

```bash
rosservice call /custom/cv/detectron/detect "{}"
```

Ihr könnt den Service auch in einem Python-Skript aufrufen. Hier ist ein Beispiel, wie Sie den Service aufrufen und die Antwort verarbeiten können:

```python
import rospy
from custom_interfaces.srv import Detectron
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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
        print(f"Bounding Box: xmin={bbox.xmin}, ymin={bbox.ymin}, xmax={bbox.xmax}, ymax={bbox.ymax}, class_id={bbox.class_id}, class_name={bbox.class_name}")

except rospy.ServiceException as e:
    print(f"Service call failed: {e}")
```

Die Antwort des Detectron Service besteht aus folgenden Daten:

    image: Ein ROS Image-Nachrichtentyp (sensor_msgs/Image), der das annotierte Bild enthält.
    bounding_boxes: Eine Liste von BoundingBox Nachrichten, wobei jede BoundingBox folgende Felder hat:
        xmin: Die x-Koordinate des linken unteren Punktes der Bounding Box.
        ymin: Die y-Koordinate des linken unteren Punktes der Bounding Box.
        xmax: Die x-Koordinate des rechten oberen Punktes der Bounding Box.
        ymax: Die y-Koordinate des rechten oberen Punktes der Bounding Box.
        class_id: Die ID der erkannten Klasse des Objekts in der Bounding Box.
        class_name: Die Bezeichnung der erkannten Klasse.

## Hinweise
### Allgemein
Die Eigenschaften der Umgebung, in der QTrobot eingesetzt wird, haben einen großen Einfluss auf die Leistung der Module wie Gesichtserkennung und Spracherkennung.
QTrobot funktioniert am besten in einem Raum mit wenigen Menschen, geringem Geräuschpegel und ohne Gegenlicht.
Zum Beispiel kann QTrobot Schwierigkeiten haben, Personen vor einem Fenster zu erkennen.

### Stabilität
In der Robotik kann es schnell passieren, dass viele Prozesse gleichzeitig laufen.
Das führt dazu, dass der Roboter langsamer wird, oder dass sogar gewisse Prozesse blockiert werden.
Es ist daher ratsam, an den richtigen Stellen `time.sleep()` einzufügen, damit der Roboter nicht überlastet wird 
und ein stabiles Verhalten aufweist. Die richtigen Stellen zu finden, ist jedoch nicht immer einfach. 
Etwas experimentieren ist daher ratsam.

### QTrobot Fähigkeiten
Im Ordner qt_skills finden sich alle Möglichkeiten für Gesten, Emotionen etc.
Dies kann sehr nützlich sein, denn die Dateibezeichnung ist jeweils der Name der Funktion,
die via rosservice aufgerufen werden kann.

### Videostream abfangen
Um den Videostream abzufangen, steht ein rostopic zur Verfügung.
Dieser kann wie gewohnt mit `rostopic echo /qt_robot/camera/front/image_raw` abgefangen werden.
Oder alternativ mit einem Python Script. Damit das Bild in Python verwendet werden kann,
müssen noch einige Schritte durchgeführt werden. Diese sind im folgenden Code aufgeführt.
```python
import numpy as np

def image_callback(msg):
   height, width = msg.height, msg.width
   img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3) % 256
   return img
```

## Weiterführende Links

**Luxai Dokumentation und Tutorials**
* [Einführung in ROS](https://docs.luxai.com/docs/tutorials/intro_ros)
* [Python Tutorial](https://docs.luxai.com/docs/tutorials/python/python_ros_project)

**Luxai Github**
* [Luxai Github](https://github.com/luxai-qtrobot/tutorials)
* [Beispielprogramme](https://github.com/luxai-qtrobot/tutorials/tree/master/examples)
* [Demos](https://github.com/luxai-qtrobot/tutorials/tree/master/demos)

**Notebooks**
* [Einführung in ROS](introduction_ROS.ipynb)