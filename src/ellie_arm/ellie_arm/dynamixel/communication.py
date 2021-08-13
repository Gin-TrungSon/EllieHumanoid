from dynamixel_sdk import *
from contextlib import contextmanager
import logging
logger = logging.getLogger(__name__)

from ultis import *


class Dxl:
    __used_ports = set()
    __controls = []
    _protocol = 1.0

    @classmethod
    def get_used_ports(cls):
        return list(cls.__used_ports)

    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, time_out=0.05, use_sync_read=False):
        self.open(port, baudrate, time_out)

    def open(self, port, baudrate, time_out):
        self.port = PortHandler(port)
        if self.port.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            exit()

        self.packetHandler = PacketHandler(self._protocol)

    def factory_reset(self, dxl_id):
        """
        resets all settings of Dynamixel to default values .incl ID, Baudrate
        """
        self.packetHandler.factoryReset(self.port, dxl_id)

    def scan(self, ids=range(254)):

        return [id for id in ids if self.ping(id)]

    def ping(self, dxl_id):
        _, result, _ = self.packetHandler.ping(self.port, dxl_id)
        return True if result == COMM_SUCCESS else False

    def get_model(self, dxl_id):
        return self.packetHandler.read2ByteTxRx(self.port, dxl_id, MODEL_NUMBER)

    def get_firmware_version(self, dxl_id):
        return self.packetHandler.read1ByteTxRx(self.port, dxl_id, FIRMWARE_VERSION)

    def change_id(self, dxl_id, new_dxl_id):
        if not self.ping(new_dxl_id):
            self.packetHandler.write1ByteTxRx(
                self.port, dxl_id, ID, new_dxl_id)
        else:
            raise ValueError(f'id {new_dxl_id} is already used.')

    def change_baudrate(self, dxl_id, new_baurate):
        if new_baurate in AVAILABLE_BAUDRATE:
            self.packetHandler.write1ByteTxRx(
                self.port, dxl_id, BAUDRATE, new_baurate)
        else:
            raise ValueError(f' baurate {new_baurate} is not supported')

    def get_status_return_level(self, dxl_id):
        return self.packetHandler.read1ByteTxRx(self.port, dxl_id, STATUS_RETURN_LEVEL)

    def get_pid_gain(self, dxl_id):
        return [self.packetHandler.read1ByteTxRx(self.port, dxl_id, P_GAIN),
                self.packetHandler.read1ByteTxRx(self.port, dxl_id, I_GAIN),
                self.packetHandler.read1ByteTxRx(self.port, dxl_id, D_GAIN), ]

    def set_pid_gain(self, dxl_id, P, I, D):
        self.packetHandler.writeByteTxRx(self.port, dxl_id, P_GAIN, P)
        self.packetHandler.writeByteTxRx(self.port, dxl_id, I_GAIN, I)
        self.packetHandler.writeByteTxRx(self.port, dxl_id, D_GAIN, D)

    def switch_led_on(self, dxl_id):
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, STATUS_LED, 1)

    def switch_led_off(self, dxl_id):
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, STATUS_LED, 0)

    def enable_torque(self, dxl_id):
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, TORQUE_ENABLE, 1)

    def disable_torque(self, dxl_id):
        self.packetHandler.write1ByteTxRx(self.port, dxl_id, TORQUE_ENABLE, 1)

    def set_goal(self,dxl_id,value):
        model_type = dynamixelModels[self.get_model(dxl_id)[0]]
        dxl_value = degree_to_dxl(value,model_type)
        self.packetHandler.write2ByteTxRx(self.port,dxl_id,GOAL_POSITION,dxl_value)


if __name__ == "__main__":
    x = Dxl()
    print(x.get_firmware_version(13))
    print(x.get_model(13))
    # print(x.scan())
    input("swith led on")
    x.switch_led_on(13)
    input("swith led off")
    x.switch_led_off(13)
    while True:
        try:
            value = float(input("goal degree : "))
            x.set_goal(13,value)
        except:
            break
        
