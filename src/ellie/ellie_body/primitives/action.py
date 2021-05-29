import time
from pypot.primitive.move import Move, MovePlayer

from pypot.primitive.primitive import Primitive 
class EllieAction(Primitive):

    def __init__(self, robot, action = None):
        self.ellie_body = robot
        self.action = action
        super().__init__(robot)

    def run(self):

        try:
            self.ellie_body.upper_body_idle_motion.stop()
            self.ellie_body.torso_idle_motion.stop()
        except:
            pass
        
        with open(f"src/body/actions/{self.action}.move") as m:
            motion_cfg = Move.load(m)
        
        # start the motion
        motion = MovePlayer(self.ellie_body,motion_cfg)
        motion.start()

        duration = motion.duration()-0.3
        time.sleep(duration)
    # function that is call once the run function has ended
    def teardown(self):
        try:
            self.ellie_body.upper_body_idle_motion.stop()
            self.ellie_body.torso_idle_motion.stop()
        except:
            pass