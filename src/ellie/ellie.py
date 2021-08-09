import sys
from threading import Thread
sys.path.append("")
from src.ellie.ellie_ears.ellie_ears import EllieEars
from src.ellie.ellie_voice.ellie_voice import EllieVoice
from src.ellie.ellie_screen.ellie_screen import EllieScreen
from src.ellie.ellie_eyes.ellie_eyes import EllieEyes
from time import sleep, time
from multiprocessing import Process
import json
import os

#from src.ellie.ellie_body.ellie_body import EllieBody


class Ellie:

    def __init__(self) -> None:
        self.ellie_ears = EllieEars()
        #self.ellie_eyes = EllieEyes()
        self.ellie_voice = EllieVoice()
        #self.eyes_thread = Thread(target=self.__eyes_loop,daemon=True)
        #self.eyes_thread.start()
        #self.screen_thread = Thread(target=self.__screen_thread,daemon=True)
        #self.screen_thread.start()
        #self.ellie_body = EllieBody()
        
        #self.intents = self.loadBehaviors()
        self.period = 1.0
        while True:
            try:
            
                startLoop = time()
                self.update()
                # self.lateUpdate()
                # duration = time()-startLoop
                # if duration < self.period:
                #     sleep(self.period - duration)
            except KeyboardInterrupt:
                #self.ellie_eyes.on_exit()
                #self.eyes_thread.join()
                #self.screen_thread.join()
                break

    def update(self):
        self.p_ears = self.ellie_ears.update()
        if self.ellie_ears.get_response() != "":
            self.ellie_voice.speak(self.ellie_ears.get_response())

    def lateUpdate(self):
        in_context = False
        for i in self.intents:
            if self.ellie_ears.get_request_text().lower() == i["listen"].lower() or (self.isAnalyzing == False and self.ellie_eyes.get_registeredPerson().__contains__(i["see"].lower())):
                #self.p_body = Process(self.__process_body(), args=i).start()
                self.p_eyes = Process(self.__process_eyes(), args=i)
                self.p_screen = Process(
                    self.__process_screen(), args=i)
                self.p_voice = Process(self.__process_voice(), args=i)
                self.p_eyes.start()
                self.p_screen.start()
                self.p_voice.start()
                in_context = True
                break
        if in_context and self.ellie_ears.get_response() != "":
            print(self.ellie_ears.get_response())
            self.p_voice = Process(
                self.__process_voice_speak(), args=self.ellie_ears.get_response())
            self.p_voice.start()
        self.__processingJoin()

    def __eyes_loop(self):
        while True:
            self.ellie_eyes.update()
            print(self.ellie_eyes.get_objects_in_frame())

    def __screen_thread(self):
        self.ellie_screen = EllieScreen()
    def cam_analyze(self):
        self.isAnalyzing = True
        self.ellie_eyes.update()
        self.isAnalyzing = False

    def __processingJoin(self):
        # if self.p_body != None:
        #    self.p_body.join()
        if self.p_screen != None:
            self.p_screen.join()
        if self.p_voice != None:
            self.p_voice.join()

    # def __process_body(self, element):
    #     if element["body_motion"] != "":
    #         self.ellie_body.do(element["body_motion"])

    def __process_eyes(self, element):
        if element["eyes_motion"] != "":
            self.ellie_screen.changeEyesMotion(element["eyes_motion"])

    def __process_screen(self, element):
        if element["url"] != "":
            self.ellie_screen.open(element["url"])

    def __process_voice(self, element):
        if element["speak"] != "":
            self.ellie_voice.speak(element["speak"])

    def __process_voice_speak(self, text):
        if text != "":
            self.ellie_voice.speak(text)

    def loadBehaviors(self):
        path = f"src/ellie/behaviors/behaviors.json"
        if not os.path.exists(path):
            return
        with open(path, "r") as write_file:
            parsed_json = json.load(write_file)
            return parsed_json["intents"]


if __name__ == "__main__":
    e = Ellie()
