import sys
sys.path.append("")
import  src.ellie.ellie_eyes.face_recognition.factical_recognition as factical_recognition
from  src.ellie.ellie_eyes.ellie_eyes_utils import Stream
import cv2.cv2 as cv2
import numpy as np
import time
from threading import Thread
import os


DATA_PATH = os.path.join(os.path.dirname(__file__),"data")
faces, names = factical_recognition.load_images(DATA_PATH)
processing = False
class FaceRecognition():

    def __init__(self):
        pass

    def detec_from_cam(self):
        # take video frames
        stream = Stream().start()

        while (stream.isOpened()):
            try:
                frame = stream.get_frame()
                # read a fream
                frame, _ ,_ = self.inference(frame)

                # show out
                cv2.imshow('Face Recognition', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                break
        cv2.destroyAllWindows()
        stream.stop()

    def inference(self,frame):

        start = time.time()
        try:
            frame_img = cv2.resize(frame, (0, 0), None, 0.25, 0.25)
        except:
            return None, None,None
        frame_img = cv2.cvtColor(frame_img, cv2.COLOR_BGR2RGB)

        faces_location = factical_recognition.detect_face_locations(frame_img)
        encoded_faces = factical_recognition.face_encodings(
            frame_img, faces_location)
        #print("EncodeTime: {}".format(time.time()-start))
        detected = []
        centers =[]
        for encodedFace, faceLocation in zip(encoded_faces, faces_location):
            matches, distances = factical_recognition.compare_faces(
                faces, encodedFace)
            matchIndex = np.argmin(distances)
            if matches[matchIndex]:
                name = names[matchIndex]
                bb = faceLocation
                y1, x2, y2, x1 = faceLocation
                y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(frame, (x1, y2-35), (x2, y2),
                                (0, 255, 0), cv2.FILLED)
                cv2.putText(frame, f"{name} {np.round(1-distances[matchIndex],2)} ", (x1+6, y2-6),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
                detected.append(name.lower())
                centers.append(((x1+x2)*0.5, (y1+y1)*0.5))
            else:
                name = "unknow"
                bb = faceLocation
                y1, x2, y2, x1 = faceLocation
                y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(frame, (x1, y2-35), (x2, y2),
                                (0, 255, 0), cv2.FILLED)
                cv2.putText(frame, f"{name} ", (x1+6, y2-6),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
                #detected.append(name)
                #centers.append(((x1+x2)*0.5, (y1+y2)*0.5))
                
        #print("FPS: {}".format(fps))
        return frame,detected,centers

if __name__ == "__main__":
    f =FaceRecognition()
    f.detec_from_cam()
