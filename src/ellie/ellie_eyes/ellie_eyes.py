import sys
from time import sleep
sys.path.append("")
from src.ellie.ellie_behavior import EllieBehavior
from src.ellie.ellie_eyes.face_recognition.factical_face_rec import FaceRecognition
from src.ellie.ellie_eyes.object_detection_lite.lite_detection import Lite_Detector
from src.ellie.ellie_eyes.ellie_eyes_utils import FPS, Stream
from imutils.video import VideoStream
from threading import Thread
import cv2.cv2 as cv2
import time
class EllieEyes(EllieBehavior):
    def __init__(self,display = False) :
        super().__init__()
        self.display = display
        self.resolution = (320,240)
        self.usePicamera = False
        self.face_recogition = FaceRecognition()
        self.object_detection = Lite_Detector()
        if self.usePicamera:
            self.stream = VideoStream(usePiCamera=True, resolution=self.resolution, framerate=24)
        else:
            self.stream = VideoStream(resolution=self.resolution,framerate=24)
        #self.stream= Stream()
        self._obj_boxes = []
        self._obj_classes=[]
        self._registeredPerson=[]
        self._centers=[]
        self._stopped = False
        self.open()
        sleep(2)
    
    def open(self):
        self.stream.start() 
        #cv2.startWindowThread()
        #self._eyes_thread = Thread(target=self.__updateTheard, args=())
        #self._eyes_thread.daemon= True
        #self._eyes_thread.start()
        #pass
    
    def update(self):
        frame = self.stream.read()
        self._obj_frame, self._obj_boxes, self._obj_classes, self.scores = self.object_detection.inference(frame)
        self.face_frame, self._registeredPerson, self.centers = self.face_recogition.inference(frame)
        if self.display:
            f = self.object_detection.draw_bbox( self.face_frame, self._obj_boxes, self._obj_classes, self.scores)
            cv2.imshow("frame",f)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows() 

    def __updateTheard(self):
        while True:
            self.update()

    def on_exit(self):
        self.stream.stop()
        self._close()

    def _close(self):
        self._stopped = True
        self._eyes_thread.join()

    def get_registeredPerson(self):
        return self._registeredPerson
    
    def get_objects_in_frame(self):
        return self._obj_classes

    def has_someone_in_frame(self):
        return "person" in self._obj_classes
    
    def nof_people_in_frame(self):
        return len([i for i in self._obj_classes if i =="person"])        

if __name__ =="__main__":
    e = EllieEyes(display=True)
    while True:
        start = time.time()
        e.update()
        #print(e.get_objects_in_frame())
        #print(time.time()-start)
        time.sleep(1)
