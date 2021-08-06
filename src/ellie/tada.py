import sys
sys.path.append("")
from multiprocessing import Process
from time import sleep, time
from src.ellie.ellie_eyes.ellie_eyes import EllieEyes
from src.ellie.ellie_screen.ellie_screen import EllieScreen
class Tada:
    def __init__(self) -> None:
        self.ellie_eyes = EllieEyes()
        while True:
            startLoop = time()
            self.update()
            #self.lateUpdate()
            duration = time()-startLoop
            if duration <  self.period:
                sleep(self.period -duration)


    def update(self):
        self.p_eyes = Process(self.ellie_eyes.update()).start()
if __name__=="__main__":
    e = Tada()