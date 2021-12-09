# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.


import os
import time
import numpy as np
import sys
sys.path.append("")
import cv2 as cv2
from ellie_eyes.ellie_eyes_utils import Stream
import ellie_eyes.face_recognition.factical_recognition as factical_recognition


#Path to encoded face data
DATA_PATH = os.path.join(os.path.dirname(__file__), "data")
faces, names = factical_recognition.load_images(DATA_PATH)


class FaceRecognition():

    def __init__(self):
        pass

    def detec_from_cam(self):
        """
        detect from camera : Only for testing !
        """
        # take video frames
        stream = Stream().start()

        while (stream.isOpened()):
            try:
                frame = stream.get_frame()
                if frame == None:
                    print("Frame None")
                    continue
                # read a fream
                frame, _, _ = self.inference(frame)

                # show out
                cv2.imshow('Face Recognition', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                break
        cv2.destroyAllWindows()
        stream.stop()

    def inference(self, frame):
        """
        Face recognition
        Parameters
        ----------
        frame: image frame 

        Returns
        -------
        img:
            handled frame
        list:
            a list of detected faces
        list:
            a list of center positions
        """
        start = time.time()
        # try to resize the image
        try:
            frame_img = cv2.resize(frame, (0, 0), None, 0.25, 0.25)
        except:
            return None, None, None

        # convert from opencv BGR color to RGB color
        frame_img = cv2.cvtColor(frame_img, cv2.COLOR_BGR2RGB)

        # ddetect facial parts position
        faces_location = factical_recognition.detect_face_locations(frame_img)

        # encode the facial parts position and save it if not exist
        encoded_faces = factical_recognition.face_encodings(
            frame_img, faces_location)

        detected = []
        centers = []

        for encodedFace, faceLocation in zip(encoded_faces, faces_location):
            matches, distances = factical_recognition.compare_faces(
                faces, encodedFace)
            matchIndex = np.argmin(distances)
            if matches[matchIndex]:
                name = names[matchIndex]
                bb = faceLocation
                # draw the bounding box at the position of detected face
                y1, x2, y2, x1 = faceLocation
                y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(frame, (x1, y2-35), (x2, y2),
                              (0, 255, 0), cv2.FILLED)
                # put the label text
                cv2.putText(frame, f"{name} {np.round(1-distances[matchIndex],2)} ", (x1+6, y2-6),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
                detected.append(name.lower())
                centers.append(((x1+x2)*0.5, (y1+y1)*0.5))
            else:
                name = "unknow"
                bb = faceLocation
                # draw the bounding box at the position of detected face
                y1, x2, y2, x1 = faceLocation
                y1, x2, y2, x1 = y1*4, x2*4, y2*4, x1*4
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(frame, (x1, y2-35), (x2, y2),
                              (0, 255, 0), cv2.FILLED)
                 # put the label text
                cv2.putText(frame, f"{name} ", (x1+6, y2-6),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)

        return frame, detected, centers


if __name__ == "__main__":
    f = FaceRecognition()
    f.detec_from_cam()
