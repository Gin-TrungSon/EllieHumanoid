import sys
from threading import Thread
import time
sys.path.append("")
from src.ellie.ellie_behavior import EllieBehavior
from src.ellie.ellie_ears.NLP_tensorflow import Inference
import speech_recognition as sr
from queue import Queue

class EllieEars(EllieBehavior):
    def __init__(self):  
        super().__init__()      
        self._ellie = Inference()
        self._recognizer = sr.Recognizer()
        self._microphone = sr.Microphone()

        """
        This will improve the recognition of the speech when working with the audio file.
        """
        with self._microphone as source:
            self._recognizer.pause_threshold = 1
            self._recognizer.adjust_for_ambient_noise(source)
        self.reset()
        self._is_busy = False

    
    def reset(self):
        self._current_response =""
        self._current_msg = ""

    def update(self):
        self.reset()
        try:
            with self._microphone as source:
                print("listening")
                audio = self._recognizer.adjust_for_ambient_noise(source)
                audio = self._recognizer.listen(source=source)
                self._current_msg = self._recognizer.recognize_google(audio,language='de-DE')
                self._current_response = self._response(self._current_msg)
        except:
            pass
    
    def get_response(self):
        return self._current_response
    
    def get_request_text(self):
        return self._current_msg


    # def _callback(self, recognizer, audio):
    #     if self._is_busy: return
    #     try:
    #         response = self._recognizer.recognize_google(audio,language="de-DE")
    #         self._current_response = self._ellie.response(response)
    #     except sr.UnknownValueError:
    #         print("I could not understand you")
    #     except sr.RequestError as e:
    #         print("Could not request results from Google Speech Recognition service; {0}".format(e))

    # def open(self):
    #     self.background_listener = self._recognizer.listen_in_background(self._microphone,self._callback)


    def _response(self, text):
        return self._ellie.response(text)
    
    def on_exit(self):
        pass

    # def stop_listening(self):
    #     self._is_busy = True
    # def start_listening(self):
    #     self._is_busy = False

if __name__=="__main__":

    e = EllieEars()
    while True:
        e.update()    
        if e.get_response() != "":  
            print(e.get_response())
        time.sleep(0.5)

