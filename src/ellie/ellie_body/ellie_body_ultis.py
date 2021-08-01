import os
import glob
import sys
import sys
sys.path.append("")
from pypot import primitive
import pathlib
sys.path.append(pathlib.Path(__file__).parent)

from src.ellie.ellie_body.primitives.action import EllieAction
from src.ellie.ellie_body.primitives.idle import HeadIdleMotion, TorsoIdleMotion, UpperBodyIdleMotion
from src.ellie.ellie_body.primitives.off import Off
from src.ellie.ellie_body.primitives.rest import Rest

from pypot import creatures
import pypot.primitive
from pypot.creatures import PoppyTorso
import time
import math
from threading import Thread
from pypot.creatures import AbstractPoppyCreature
import json
from pypot.robot import from_json
from pypot.primitive import Primitive
from pypot.primitive.move import MoveRecorder, Move, MovePlayer
import glob
import random



ACTION_DIR =os.path.join(pathlib.Path(__file__).parent,"actions/*.move") 
_primitives =[]
def get_primitives():
    """
    List of already taken primitives
    """
    if len(_primitives)<1:
        for file in glob.glob(ACTION_DIR):
            _primitives.append(os.path.splitext(file)[0]) 
    return _primitives

def reload_primitives():
    for file in glob.glob(ACTION_DIR):
        _primitives.append(os.path.splitext(file)[0]) 
    return _primitives