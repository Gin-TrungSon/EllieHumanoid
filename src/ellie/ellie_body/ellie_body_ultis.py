import os
import glob
import sys

from pypot import primitive
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),"primitives")))

from primitives.action import EllieAction
from primitives.idle import HeadIdleMotion, TorsoIdleMotion, UpperBodyIdleMotion
from primitives.off import Off
from primitives.rest import Rest

from pypot import creatures
import pypot.primitive
from pypot.creatures import PoppyTorso
import time
import math
import threading
from pypot.creatures import AbstractPoppyCreature
import json
from pypot.robot import from_json
from pypot.primitive import Primitive
from pypot.primitive.move import MoveRecorder, Move, MovePlayer
import glob
import random

_primitives =[]
def get_primitives():
    """
    List of already taken primitives
    """
    if len(_primitives)<1:
        for file in glob.glob("src/body/actions/*.move"):
            _primitives.append(os.path.splitext(file)[0]) 
    return _primitives

def reload_primitives():
    for file in glob.glob("src/body/actions/*.move"):
        _primitives.append(os.path.splitext(file)[0]) 
    return _primitives