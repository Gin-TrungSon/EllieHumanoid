import sys
import os
sys.path.append("")
import json

from multiprocessing import Process
from time import sleep, time
from src.ellie.ellie_screen.ellie_screen import EllieScreen
from src.ellie.ellie_body.ellie_body import EllieBody
from src.ellie.ellie_voice.ellie_voice import EllieVoice
from src.ellie.ellie_eyes.ellie_eyes import EllieEyes
from src.ellie.ellie_ears.ellie_ears import EllieEars


class Ellie:
    def __init__(self):
        self.ellie_ears = EllieEars()
        self.ellie_eyes = EllieEyes()
        self.ellie_voice = EllieVoice()
        self.ellie_body = EllieBody()
        self.ellie_screen = EllieScreen()
        self.intents = self.loadBehaviors()
        self.period = 1.0
        while True:
            startLoop = time()
            self.update()
            self.lateUpdate()
            duration = time()-startLoop
            if duration <  self.period:
                sleep(self.period -duration)


    def update(self):
        self.p_eyes = Process(self.ellie_eyes.update()).start()
        self.p_ears = Process(self.ellie_ears.update()).start()

    def lateUpdate(self):
        for i in self.intents:
            if self.ellie_ears.get_request_text().lower() == i["listen"].lower() or self.ellie_eyes.get_registeredPerson().__contains__(i["see"].lower()):
                self.p_body = Process(self.__process_body(), args=i).start()
                self.p_eyes = Process(self.__process_eyes(), args=i).start()
                self.p_screen = Process(
                    self.__process_screen(), args=i).start()
                self.p_voice = Process(self.__process_voice(), args=i).start()
                break
        self.__processingJoin()

    def __processingJoin(self):
        if self.p_eyes != None:
            self.p_eyes.join()
        if self.p_ears != None:
            self.p_ears.join()
        if self.p_body != None:
            self.p_body.join()
        if self.p_screen != None:
            self.p_screen.join()
        if self.p_voice != None:
            self.p_voice.join()

    def __process_body(self, element):
        if element["body_motion"] != "":
            self.ellie_body.do(element["body_motion"])

    def __process_eyes(self, element):
        if element["eyes_motion"] != "":
            self.ellie_screen.changeEyesMotion(element["eyes_motion"])

    def __process_screen(self, element):
        if element["url"] != "":
            self.ellie_screen.open(element["url"])

    def __process_voice(self, element):
        if element["speak"] != "":
            self.ellie_voice.speak(element["speak"])

    def loadBehaviors(self):
        if not os.path.exists():
            return
        with open(f"src/ellie/behaviors/behaviors.json", "r") as write_file:
            parsed_json = json.load(write_file)
            return parsed_json["intents"]

if __name__=="__main__":
    e = Ellie()