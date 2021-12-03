# Copyright 2021 by Dong Trung Son, Nueremberg University.
# Email: trungsondo68839@th-nuernberg.de
# All rights reserved.
# This file is part of the Ellie-Project,
# and is released under the "MIT License Agreement". Please see the LICENSE
# file that should have been included as part of this package.

from math import fabs
from pathlib import Path
import threading
from numpy.core.numeric import Infinity
from scipy.spatial import cKDTree
import json
import time


class KDDict(dict):
    """
        Dictionary to return the value with closest key
        https://stackoverflow.com/questions/29094458/find-integer-nearest-neighbour-in-a-dict
    """

    def __init__(self, ndims=1, regenOnAdd=False):
        super(KDDict, self).__init__()
        self.ndims = ndims
        self.regenOnAdd = regenOnAdd
        self.__keys = []
        self.__tree = None
        self.__stale = False

    # Enforce dimensionality
    def __setitem__(self, key, val):
        if not isinstance(key, tuple):
            key = (key,)
        if len(key) != self.ndims:
            raise KeyError("key must be %d dimensions" % self.ndims)
        self.__keys.append(key)
        self.__stale = True
        if self.regenOnAdd:
            self.regenTree()
        super(KDDict, self).__setitem__(key, val)

    def regenTree(self):
        self.__tree = cKDTree(self.__keys)
        self.__stale = False

    def nearest_key(self, key):
        """Find a closest time stamp

        Args:
            key (float): time stamp

        Returns:
            float: closest time stamp
        """
        if not isinstance(key, tuple):
            key = (key,)
        if self.__stale:
            self.regenTree()
        _, idx = self.__tree.query(key, 1)
        return self.__keys[idx]

    def getKeys(self):
        """Get all keys of dictionary

        Returns:
            list: keys
        """
        return self.__keys

    def __missing__(self, key):
        if not isinstance(key, tuple):
            key = (key,)
        if len(key) != self.ndims:
            raise KeyError("key must be %d dimensions" % self.ndims)
        return self[self.nearest_key(key)]


class Behavior():
    def __init__(self, id, dxl_interface, freq):
        self._id = id
        self._frame_rate = freq
        self._dxl_interface = dxl_interface
        self._stamped_positions = KDDict()
        self._recorded_positions = {}

    def setItems(self, positions):
        try:
            for key in positions.keys():
                self._stamped_positions[key] = positions[key]
        except:
            raise TypeError("can not read json file !")

    @property
    def frame_rate(self):
        """the rate to update frame

        Returns:
            float: frame rate
        """
        return self._frame_rate

    def add_position(self, position, velocity, time):
        """Add new state of motors at the stamped time

        Args:
            position (list): current motors position
            velocity (list): current motors velocity
            time (float): time stamp
        """
        self._stamped_positions[time] = [position, velocity]

    @property
    def stamped_positions(self):
        """Get all positions with time stamp

        Returns:
            KDict: positions
        """
        return self._stamped_positions

    @property
    def id(self):
        """Get Behavior Id

        Returns:
            string: Behavior name
        """
        return self._id

    def get_position(self, key):
        """Get nearest position at a time stamp

        Args:
            key (float): time stamp
        """
        self.stamped_positions[self.stamped_positions.nearest_key(key)]

    @classmethod
    def create(cls, id, dxl_interface, d):
        """Create new behavior from recorded file

        Args:
            id (string): Behavior name
            dxl_interface (DxlInterface): Dynamixel communication interface
            d (OrderedDict): A Dictionary that contains [framerate] and [posittion] element

        Returns:
            Behavior: new behavior from file
        """
        behavior = cls(id, dxl_interface, d['framerate'])
        behavior.setItems(d['positions'])
        return behavior

    @classmethod
    def load(cls, id, dxl_interface, file):
        """Load Behavior as a dict from file

        Args:
            id (string): Behavior name
            dxl_interface (DxlInterface): Dynamixel communication interface
            file (string): json file path

        Returns:
            Behavior: new behavior from file
        """
        d = json.load(file)
        return cls.create(id, dxl_interface, d)

    @property
    def duration(self):
        """Duration

        Returns:
            float: time interval
        """
        return 1.0/float(self.frame_rate)

    def start_recorder(self):
        """Start to recode new behavior
        """
        self.running_event = threading.Event()
        self.running_event.set()
        self.recorder = threading.Thread(
            target=self._start, daemon=True)

        counter = 3
        for motor in self._dxl_interface.registered_motors:
            self._dxl_interface.disable_torque(motor.id)
        while counter > 0:
            print(f"Start in {counter}")
            time.sleep(1)
            counter -= 1

        self.recorder.start()

    def stop_recorder(self):
        """Stop the recording
        """
        self.setItems(self._recorded_positions)
        self.running_event.clear()

    def resume_recorder(self):
        """Resume the recording
        """
        self.running_event.set()

    def replay(self):
        """Replax the recorded behavior
        """
        self.execute()

    def execute(self):
        """Execute current behavior
        """
        self._dxl_interface.goto_position_sync(
            self.stamped_positions, self.duration)

    def save(self, file):
        """Save behavior file 

        Args:
            file (string): the path to save file
        """
        with open(file, "w") as f:
            self.stop = True
            self.running_event.set()
            d = {
                'framerate': self.frame_rate,
                'positions': self._recorded_positions,
            }
            json.dump(d, f, indent=4)

    def _start(self):
        self.stop = False
        time_stamped = 0.0
        period = -1
        max_period = float('-inf')
        while True:
            if not self.running_event.is_set():
                self.running_event.wait(60)
                if self.stop or not self.running_event.is_set():

                    break
            start = time.time()
            self._recorded_positions[time_stamped] = self._dxl_interface.read_data_sync(
            )
            period = time.time()-start
            if period > max_period:
                max_period = period
                self._frame_rate = 1/max_period
            time_stamped += period


# if __name__ == "__main__":
#     pass
    # with open("src/ellie/ellie_body/actions/dance_handsup.move") as f:
    #     b = Behavior.load(f)
    #     print(
    #         b.stamped_positions[b.stamped_positions.nearest_key('3.905321836')])
