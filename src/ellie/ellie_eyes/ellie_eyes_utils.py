import cv2.cv2 as cv2
from threading import Thread
try:
    from picamera.array import PiRGBArray
    from picamera import Picamera 
except:
    pass
import datetime
import time 
PI_CAMERA = False
RESOLUTION =(640,480)
class Stream:

    def __init__(self):
        self._stream = cv2.VideoCapture(0)
        self._stream.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        self._stream.set(3, RESOLUTION[0])
        self._stream.set(4, RESOLUTION[1])

        self._success, self._frame = self._stream.read()
        self._stopped = False

    def start(self):
        self._thread = Thread(target=self.update, args=())
        self._thread.daemon= True
        self._thread.start()
        return self

    def stop(self):
        self._stopped = True
        self._thread.join()

    def update(self):
        while self._stream.isOpened():
            if self._stopped:
                self._stream.release()
                return
            self._success, self._frame = self._stream.read()
            time.sleep(0.2)

    def get_frame(self):
        return self._frame

    def isOpened(self):
        if self._stopped == False:
            return self._stream.isOpened()
        else:
            return False

class PIStream:
    def __init__(self, frame_rate =32) :
        self.camera = PiCamera()
        self.camera.resolution = RESOLUTION
        self.camera.framerate = frame_rate
        self.rawCapture = PiRGBArray(self.camera, size=RESOLUTION)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)

        self.frame = None
        self.stopped = False


    def start(self):
        self._thread = Thread(target=self.update, args=())
        self._thread.daemon= True
        self._thread.start()
        return self       
    
    def update(self):
        while self._stream.isOpened():
            if self._stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
            self._success, self._frame = self._stream.read()
            time.sleep(0.2)
    
    def get_frame(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self._thread.join()

    def isOpened(self):
        if self._stopped == False:
            return self._stream.isOpened()
        else:
            return False
    
class FPS:
	def __init__(self):
		# store the start time, end time, and total number of frames
		# that were examined between the start and end intervals
		self._start = None
		self._end = None
		self._numFrames = 0
	def start(self):
		# start the timer
		self._start = datetime.datetime.now()
		return self
	def stop(self):
		# stop the timer
		self._end = datetime.datetime.now()
	def update(self):
		# increment the total number of frames examined during the
		# start and end intervals
		self._numFrames += 1
	def elapsed(self):
		# return the total number of seconds between the start and
		# end interval
		return (self._end - self._start).total_seconds()
	def fps(self):
		# compute the (approximate) frames per second
		return self._numFrames / self.elapsed()