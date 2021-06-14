import sys
import os
sys.path.append("")
from gtts import gTTS
from gtts import gTTS
from pygame import mixer
import mutagen.mp3
import os
import glob
import pathlib
import time
from pydub import AudioSegment
from pydub.playback import play
from pydub import effects

DIR_TEMP = os.path.join(pathlib.Path(__file__).parent,"temp")
class EllieVoice:
    def __init__(self):
        filelist = glob.glob(os.path.join(DIR_TEMP, "*"))
        for f in filelist:
            os.remove(f)

    def speak(self,text):
        tts = gTTS(text, lang="de",slow=False)
        file_name=os.path.join(DIR_TEMP, "{}.mp3".format(time.time()))
        tts.save(file_name)  
        audio = AudioSegment.from_mp3(file_name)
        audio = effects.speedup(audio,1.2)
        play(audio)
        # speech_file = file_name
        # mp3 = mutagen.mp3.MP3(speech_file)
        # mixer.init(frequency= int(mp3.info.sample_rate*2))   

        # mixer.music.load(file_name)
        # mixer.music.play()
        # while mixer.music.get_busy():
        #     self.is_busy= True
        #     time.sleep(0.2)
        # self.is_busy= False
        # mixer.quit()
        # os.remove(file_name)

    def speed_change(self,sound, speed=1.0):
        # Manually override the frame_rate. This tells the computer how many
        # samples to play per second
        sound_with_altered_frame_rate = sound._spawn(sound.raw_data, overrides={
            "frame_rate": int(sound.frame_rate * speed)
        })
        # convert the sound with altered frame rate to a standard frame rate
        # so that regular playback programs will work right. They often only
        # know how to play audio at standard frame rate (like 44.1k)
        return sound_with_altered_frame_rate.set_frame_rate(sound.frame_rate)
    

if __name__=="__main__":
    e= EllieVoice()
    e.speak("Standardmäßig sollte es während der Installation des Betriebssystems zusammen mit dem Sprachpaket geliefert werden. Sie müssen das Sprachpaket manuell installieren, wenn Sie andere Sprachen verwenden möchten")
    

