from deepface import DeepFace
import cv2
import os
import uuid
import numpy as np
import shutil


class FaceDetector:
    def __init__(self, db_path):
        self.db_path = db_path
        if not os.path.exists(self.db_path):
            os.makedirs(self.db_path)

    def reset_database(self):
        if os.path.exists(self.db_path):
            shutil.rmtree(self.db_path)
        os.makedirs(self.db_path)

    def detect_faces(self, image_path):
        faces = DeepFace.extract_faces(img_path=image_path, detector_backend='opencv', enforce_detection=False)
        recognized_faces = dict()
        for face in faces:
            is_known, face_id = self._is_face_known(face['face'])
            if not is_known:
                face_id = self._save_face(face['face'])
            recognized_faces[face_id] = self._analyze_face(face['face'])
        return recognized_faces

    def _analyze_face(self, face):
        try:
            # Speichere das extrahierte Gesicht temporär, um es analysieren zu können
            temp_face_path = "temp_face_analyze.jpg"
            cv2.imwrite(temp_face_path, self._prepare_to_save(face))

            # Analysiere das Gesicht
            analysis = DeepFace.analyze(img_path=temp_face_path, actions=["age", "gender", "emotion", "race"])
            return analysis
        except Exception as e:
            print(f"Fehler bei der Gesichtsanalyse: {e}")
            return None
        finally:
            # Entferne das temporäre Bild
            if os.path.exists(temp_face_path):
                os.remove(temp_face_path)

    def _is_face_known(self, face):
        temp_face_path = "temp_face.jpg"
        cv2.imwrite(temp_face_path, self._prepare_to_save(face))

        try:
            recognition = DeepFace.find(img_path=temp_face_path, db_path=self.db_path, enforce_detection=False)
            print(recognition)
            print(type(recognition))

            # Prüfe, ob die Erkennung eine nicht-leere Liste enthält
            if isinstance(recognition, list) and len(recognition) > 0:
                # Zugriff auf das erste DataFrame in der Liste
                df = recognition[0]
                if not df.empty:
                    # Zugriff auf die erste Zeile des DataFrame
                    first_match = df.iloc[0]
                    known_face_id = os.path.basename(first_match['identity']).split('.')[
                        0]  # Extrahiere die ID aus dem Dateinamen
                    return True, known_face_id
        except Exception as e:
            print(f"Fehler bei der Gesichtssuche: {e}")
        finally:
            if os.path.exists(temp_face_path):
                os.remove(temp_face_path)

        return False, None

    def _prepare_to_save(self, face):
        face = face.astype(np.float32)
        face_corrected = cv2.cvtColor(face, cv2.COLOR_RGB2BGR)
        face_corrected = (face_corrected * 255).astype(np.uint8)
        return face_corrected

    def _save_face(self, face):
        face_corrected = self._prepare_to_save(face)
        face_id = str(uuid.uuid4())
        face_path = os.path.join(self.db_path, face_id + '.jpg')
        cv2.imwrite(face_path, face_corrected)
        print(f'Gesicht gespeichert: {face_path}')
        return face_id