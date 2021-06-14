import sys
from threading import Thread
sys.path.append("")
from src.ellie.ellie_behavior import EllieBehavior
class EllieScreen(EllieBehavior):
    def __init__(self) -> None:
        self._head_screen = Thread()
    
    def update(self, context):
        return super().update(context)

    

    