import sys
import os
sys.path.append("")
import multiprocessing
from threading import Thread
from gtts.tokenizer import pre_processors
import pygame
from pygame.constants import AUDIO_ALLOW_ANY_CHANGE, AUDIO_ALLOW_FREQUENCY_CHANGE
from pyttsx3 import engine
from src.ellie.ellie_ear.NLP_tensorflow import Inference
import time 
import speech_recognition as sr
from gtts import gTTS
import pyttsx3
from multiprocessing import Process, process
import asyncio
from gtts import gTTS
from pygame import mixer
import mutagen.mp3
import pyttsx3




class EllieVoice:
    def __init__(self):
        self.ellie = Inference(threshold= 0.8)
        self.queue =[]
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.is_busy = False
        ### using pyttsx3
        self.engine = pyttsx3.init()
        """Rate"""
        self.engine.setProperty("rate",150)

        """Volume"""
        self.engine.setProperty("volume",1.0)
        """Voice"""
        voices = self.engine.getProperty('voices') 
        # for voice in voices:
        #     print("Voice:")
        #     print(" - ID: %s" % voice.id)
        #     print(" - Name: %s" % voice.name)
        #     print(" - Languages: %s" % voice.languages)
        #     print(" - Gender: %s" % voice.gender)
        #     print(" - Age: %s" % voice.age)
        self.engine.setProperty("voice", voices[0].id)
    def _on_completed(self, name, completed):
        pass
    
    async def speak_async(self, text):
        await  asyncio.create_task(self.speak(text))

    def speak(self,text):
        Thread(target=self._speak,args=[text]).start()

    def _speak(self,text):
        # tts = gTTS(text, lang="de")
        # file_name= "temp{}.mp3".format(time.time())
        # tts.save(file_name)  
        # speech_file = file_name
        # mp3 = mutagen.mp3.MP3(speech_file)
        # mixer.init(frequency= int(mp3.info.sample_rate*8),size= 32, allowedchanges=AUDIO_ALLOW_ANY_CHANGE)   
        # print(int(mp3.info.sample_rate*8))     
        # mixer.music.load(file_name)
        # mixer.music.play()
        # while mixer.music.get_busy():
        #     self.is_busy= True
        #     time.sleep(0.2)
        # self.is_busy= False
        # mixer.quit()
        # os.remove(file_name)
        self.engine.startLoop(False)
        self.engine.say(text)
        self.engine.iterate()
        self.engine.endLoop()
        
       
    def response(self, text):
        return self.ellie.response(text)

    def _call_back(self,recognizer, audio):
        #if self.is_busy : return
        print("ok")
        try :
            info = recognizer.recognize_google(audio,language="de-DE")
            print(info)
            if info=="":
                return
            s = time.time()
            response = self.response(info)
            print("{}   {}".format(response,time.time()-s))
            if response is not None:
                self.queue.append(response)
                self._speak(response)
        except Exception as e:
            print(e)

    def open(self):
        self.stop_listening = self.recognizer.listen_in_background(self.microphone,self._call_back)
        

        
    def close(self):
        self.stop_listening(wait_for_stop=False)
        self.speak_thread.join()



if __name__=="__main__":
    e= EllieVoice()
    e.open()
    while True:
        time.sleep(1)
    #e.speak("Standardmäßig sollte es während der Installation des Betriebssystems zusammen mit dem Sprachpaket geliefert werden. Sie müssen das Sprachpaket manuell installieren, wenn Sie andere Sprachen verwenden möchten")
    

