import sys
sys.path.append("")
from src.ellie.ellie_behavior import EllieBehavior
from src.ellie.ellie_eyes.face_recognition.factical_face_rec import FaceRecognition
from src.ellie.ellie_eyes.object_detection_lite.lite_detection import Lite_Detector
from src.ellie.ellie_eyes.ellie_eyes_utils import *

class EllieEyes(EllieBehavior):
    def __init__(self,display = False) :
        super().__init__()
        self.display = display
        self.face_recogition = FaceRecognition()
        self.object_detection = Lite_Detector()
        if PI_CAMERA:
            self.stream = PIStream()
        else:
            self.stream = Stream()

        self._obj_boxes = []
        self._obj_classes=[]
        self._acquaintances=[]
        self._centers=[]
        self._stopped = False
    
    def open(self):
        self.stream.start() 
        self._eyes_thread = Thread(target=self.update, args=())
        self._eyes_thread.daemon= True
        self._eyes_thread.start()
    
    def update(self, context):
        while not self._stopped and self.stream.isOpened():
            try:
                frame = self.stream.get_frame()
                self._obj_frame, self._obj_boxes, self._obj_classes, self.scores = self.object_detection.inference(frame)
                self.face_frame, self._acquaintances, self.centers = self.face_recogition.inference(frame)
                if self.display:
                    f = self.object_detection.draw_bbox( self.face_frame, self._obj_boxes, self._obj_classes, self.scores)
                    cv2.imshow("frame",f)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except KeyboardInterrupt:
                break
        self.stream.stop()

    def close(self):
        self._stopped = True
        self._eyes_thread.join()

    def get_acquaintances(self):
        return self._acquaintances
    
    def get_objects_in_frame(self):
        return self._obj_classes

    def has_someone_in_frame(self):
        return "person" in self._obj_classes
    
    def nof_people_in_frane(self):
        return len([i for i in self._obj_classes if i =="person"])

    def isOpened(self):
        return self.stream.isOpened()        

if __name__ =="__main__":
    e = EllieEyes(display=True)
    e.open()
    while True:
        print(e.get_objects_in_frame())
        time.sleep(1)
