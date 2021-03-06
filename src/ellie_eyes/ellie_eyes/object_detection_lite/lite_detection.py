# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

from imutils.video import VideoStream, FPS
import importlib.util
import time
import cv2
import numpy as np
import sys
import os
sys.path.append("")

try:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
except:
    pass


USE_TPU = False  # Coral Edge TPU
RESOLUTION = (640, 480)
# We do not use PI camera in this project
PI_CAMERA = False

if importlib.util.find_spec('tflite_runtime'):
    from tflite_runtime.interpreter import Interpreter
    if USE_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if USE_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

LABELS_PATH = os.path.join(os.path.dirname(__file__), "tmp/labelmap.txt")
with open(LABELS_PATH, 'r') as f:
    LABElS = [line.strip() for line in f.readlines()]
if USE_TPU:
    MODEL_PATH = os.path.join(os.path.dirname(
        __file__), "tmp/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite")
    interpreter = Interpreter(model_path=MODEL_PATH,
                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
else:
    MODEL_PATH = os.path.join(os.path.dirname(__file__), "tmp/detect.tflite")
    interpreter = Interpreter(model_path=MODEL_PATH)


MEAN = 128
STD = 128


class Lite_Detector():
    def __init__(self, threshold=0.6):
        self.threshold = threshold
        interpreter.allocate_tensors()
        self.input_details = interpreter.get_input_details()
        self.output_details = interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.float_model = (self.input_details[0]['dtype'] == np.float32)

    def inference(self, frame):
        """
        Object detection: detect objects in Coco dataset
        Parameters
        ----------
        frame: image frame 

        Returns
        -------
        frame:
            handled frame
        list:
            a list of bounding boxes
        list:
            a list of labels
        list:
            a list of scores
        """
        try:
            handled_frame = cv2.resize(frame, (self.width, self.height),
                                       interpolation=cv2.INTER_AREA)
        except:
            return None, None, None, None
        input_data = np.expand_dims(handled_frame, axis=0)

        # Normalize if its a floating model
        if self.float_model:
            input_data = (np.float32(input_data) - MEAN) / STD

        interpreter.set_tensor(self.input_details[0]['index'], input_data)
        interpreter.invoke()

        # results
        # Bounding box
        boxes = interpreter.get_tensor(self.output_details[0]['index'])[0]
        # Class
        classes = interpreter.get_tensor(
            self.output_details[1]['index'])[0]
        # Confidence
        scores = interpreter.get_tensor(self.output_details[2]['index'])[0]
        rboxes = []
        rclasses = []
        rscores = []
        for i in range(len(scores)):
            if scores[i] >= self.threshold and LABElS[int(classes[i])] != "???":
                rscores.append(scores[i])
                rclasses.append(LABElS[int(classes[i])])
                rboxes.append(boxes[i])

        return frame, rboxes, rclasses, rscores

    def detect_from_cam(self):
        """
        Detect objects direct from camera: Only for testing !
        """
        if PI_CAMERA:
            stream = VideoStream(usePiCamera=True, resolution=RESOLUTION)
            fps = FPS().start()
            while stream.isOpened():
                try:
                    start = time.time()
                    frame = stream.get_frame()
                    frame, boxes, classes, scores = self.inference(frame)
                    frame = self.draw_bbox(frame, boxes, classes, scores)
                    frame_rate = 1/(time.time()-start+0.001)
                    cv2.putText(frame, 'FPS: {0:.2f}'.format(
                        frame_rate), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
                    cv2.imshow('lite_detector', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except KeyboardInterrupt:
                    cv2.destroyAllWindows()
                    break
        else:
            # start new thread
            stream = VideoStream(resolution=RESOLUTION).start()
            time.sleep(1)
            while stream.isOpened():
                try:
                    start = time.time()
                    # preprocessing
                    origin_frame = stream.get_frame()
                    frame = origin_frame.copy()
                    #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                    frame, boxes, classes, scores = self.inference(frame)
                    frame = self.draw_bbox(frame, boxes, classes, scores)
                    frame_rate = 1/(time.time()-start+0.001)
                    cv2.putText(frame, 'FPS: {0:.2f}'.format(
                        frame_rate), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
                    cv2.imshow('lite_detector', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except KeyboardInterrupt:
                    cv2.destroyAllWindows()
                    break
        cv2.destroyAllWindows()
        stream.stop()

    def draw_bbox(self, frame, boxes, input_classes, scores):
        """
        draw bounding boxes to the frame

        Parameters
        ----------
        frame: 
            image frame 
        boxes:
            bounding box positions
        input_classes: 
            a list of lables
        scores: 
            a list of scores. If score lower than the theshhold, the bounding box will be ignored
        Returns
        -------
        frame:
            handled frame
        list:
            a list of center points
        list:
            a list of labels
        """
        if frame.all() == None:
            return frame
        centers = []
        classes = []
        for i in range(len(scores)):
            if scores[i] >= self.threshold and (input_classes[i]) != "???":

                # return bbox fits to original image
                ymin = int(max(1, (boxes[i][0] * RESOLUTION[1])))
                xmin = int(max(1, (boxes[i][1] * RESOLUTION[0])))
                ymax = int(min(RESOLUTION[1], (boxes[i][2] * RESOLUTION[1])))
                xmax = int(min(RESOLUTION[0], (boxes[i][3] * RESOLUTION[0])))

                cv2.rectangle(frame, (xmin, ymin),
                              (xmax, ymax), (0, 255, 0), 2)
                cv2.rectangle(frame, (xmin, ymax-35),
                              (xmax, ymax), (0, 255, 0), cv2.FILLED)
                cv2.putText(frame, "{} {:.2f}".format(input_classes[i], scores[i]), (xmin+6, ymax-6),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
                center = ((xmax-xmin)*0.5, (ymax-ymin)*0.5)
                centers.append(center)
                classes.append(input_classes[i])
        return frame, centers, classes


if __name__ == "__main__":
    l = Lite_Detector()
