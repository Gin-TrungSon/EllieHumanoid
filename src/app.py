from context import EllieContext
from ellie.ellie_body.ellie_body import EllieBody
from ellie.ellie_ears.ellie_ears import EllieEars
from ellie.ellie_ears.ellie_voice import EllieVoice
from ellie.ellie_eyes.ellie_eyes import EllieEyes
from ellie.ellie_screen.ellie_screen import EllieScreen


from multiprocessing import Process
class Ellie:
    def __init__(self) :
        self._ellie_eyes = EllieEyes()
        self._ellie_ears = EllieEars()
        self._ellie_body = EllieBody()
        self._ellie_voice = EllieVoice()
        self._ellie_screen = EllieScreen()
        self._context = EllieContext()

    def update(self):
        self._ellie_eyes.update(self._context)
        self._ellie_ears.update(self._context)

        actions = Process(target=self._ellie_body, args=(self._context))
        speaker = Process(target=self._ellie_voice, args=(self._context))
        screens = Process(target=self._ellie_screen, args=(self._context))
        actions.start()
        speaker.start()
        screens.start()

        actions.join()
        speaker.join()
        screens.join()
