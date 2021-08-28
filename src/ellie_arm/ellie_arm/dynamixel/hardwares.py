from ellie_arm.dynamixel.ultis import *


class Motor:
    def __init__(self, name, id, model, offset, direct=True, angle_limit=[-90, 90], compliant=False, safe_compliant=False):
        self._name = name
        self._id = id
        self._type = model
        self._offset = offset
        self._angle_limit = angle_limit
        self._direct = direct
        self._compliant = compliant
        self._safe_compliant = safe_compliant

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    @property
    def angle_limit(self):
        return self._angle_limit

    @property
    def type(self):
        return self._type

    @property
    def offset(self):
        return self._offset

    @property
    def direct(self):
        return self._direct
