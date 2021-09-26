# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.
from ament_index_python.packages import get_package_share_directory
import time
from threading import Thread
import rclpy
from rclpy.action.server import ActionServer
from rclpy.node import Node
from ellie_voice.NLP_tensorflow import Inference
import speech_recognition as sr
from ellie_msgs.srv import String
from ellie_msgs.msg import Conversation
from gtts import gTTS
import os
import glob
import time
from pydub import AudioSegment
from pydub.playback import play
from pydub import effects

DIR_TEMP = os.path.join(os.path.dirname(__file__), "temp")


class EllieVoice(Node):
    def __init__(self):
        super().__init__("ellie_ears")
        self.declare_parameter("pause_threshold", 1.0)
        self.declare_parameter("speech_speed", 1.3)
        self.declare_parameter("minimum_energy_threshold", 300)

        self._listen_response = self.create_publisher(
            Conversation, 'communication', 1)
        self._speak = self.create_service(String, "speak", self.speak_callback)

        print("Prepare for listening ...")
        self.remove_existed_sounds()

        self._ellie = Inference()
        self._recognizer = sr.Recognizer()
        self._microphone = sr.Microphone()

        """
        This will improve the recognition of the speech when working with the audio file.
        """
        with self._microphone as source:
            self._recognizer.energy_threshold = self.energy_threshold
            self._recognizer.pause_threshold = self.pause_threshold
            self._recognizer.adjust_for_ambient_noise(source)
        self.reset()

    @property
    def pause_threshold(self) :
        return self.get_parameter(
            "pause_threshold").get_parameter_value().double_value
    @property
    def speech_speed(self) :
        return self.get_parameter(
            "speech_speed").get_parameter_value().double_value
    @property
    def energy_threshold(self) :
        return self.get_parameter(
            "minimum_energy_threshold").get_parameter_value().integer_value

    def speak_callback(self, request, response):
        print(f"received : {request.request}")
        response.response = request.request
        self.speak(request.request)
        return response

    def remove_existed_sounds(self):
        filelist = glob.glob(os.path.join(DIR_TEMP, "*"))
        if len(filelist) < 1:
            return
        for f in filelist:
            os.remove(f)

    def reset(self):
        self._current_response = ""
        self._current_msg = ""

    def update(self):
        self.reset()
        try:
            with self._microphone as source:
                print("listening")
                audio = self._recognizer.adjust_for_ambient_noise(source)
                audio = self._recognizer.listen(source=source)
                msg = Conversation()
                self._current_msg = self._recognizer.recognize_google(
                    audio, language='de-DE')
                if self._current_msg != "":
                    msg.listen = self._current_msg

                self._current_response = self._response(self._current_msg)
                if self._current_response != "":
                    msg.response = self._current_response
                    self.speak(self._current_response)
                self._listen_response.publish(msg)
        except sr.UnknownValueError:
            print("Oops! Didn't catch that")
        except sr.RequestError as e:
            print(e)

    def get_response(self):
        return self._current_response

    def get_request_text(self):
        return self._current_msg

    def speak(self, text):
        tts = gTTS(text, lang="de", slow=False)
        file_name = os.path.join(DIR_TEMP, f"{format(time.time())}.mp3")
        tts.save(file_name)
        audio = AudioSegment.from_mp3(file_name)
        audio = effects.speedup(audio, self.speech_speed)
        play(audio)
        os.remove(file_name)

    def _response(self, text):
        return self._ellie.response(text)


def main(args=None):
    rclpy.init(args=args)
    ellie_voice = EllieVoice()
    # spin Ros in other thread
    thread_ = Thread(
        target=ros_shutdown, args=(ellie_voice,), daemon=True)
    thread_.start()
    while rclpy.ok():
        ellie_voice.update()
    ellie_voice.remove_existed_sounds()

def ros_shutdown(node):
    print("ros_spins")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
