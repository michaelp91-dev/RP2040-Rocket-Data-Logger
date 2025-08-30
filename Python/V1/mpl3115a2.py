# mpl3115a2.py
from machine import I2C
from struct import unpack
from micropython import const

MPL3115A2_ADDRESS = const(0x60)
MPL3115A2_WHOAMI = const(0x0C)
MPL3115A2_ID = const(0xC4)
MPL3115A2_CTRL_REG1 = const(0x26)
MPL3115A2_PT_DATA_CFG = const(0x13)
MPL3115A2_STATUS = const(0x00)
MPL3115A2_OUT_P_MSB = const(0x01)
MPL3115A2_OUT_T_MSB = const(0x04)

class MPL3115A2:
    def __init__(self, i2c_bus):
        self._i2c = i2c_bus
        self._address = MPL3115A2_ADDRESS

    def _test_i2c_device(self):
        try:
            return self._address in self._i2c.scan()
        except OSError:
            return False

    def init(self):
        if not self._test_i2c_device():
            print("MPL3115A2 I2C Fail!")
            return False
        self._i2c.writeto(self._address, bytes([MPL3115A2_WHOAMI]), False)
        try:
            data = self._i2c.readfrom(self._address, 1)
            sensor_id = data[0]
        except OSError:
            sensor_id = 0
        if sensor_id != MPL3115A2_ID:
            print("MPL3115A2 not found! ID:", hex(sensor_id))
            return False
        print("MPL3115A2 OK!")
        self._i2c.writeto_mem(self._address, MPL3115A2_CTRL_REG1, bytes([0b00011010]))
        return True

    def get_raw_data(self):
        """Reads raw pressure and temperature data."""
        self._i2c.writeto(self._address, bytes([MPL3115A2_OUT_P_MSB]), False)
        data = self._i2c.readfrom(self._address, 5)

        # Pressure
        adc_P = (data[0] << 16) | (data[1] << 8) | data[2]
        adc_P >>= 4

        # Temperature
        adc_T = (data[3] << 8) | data[4]
        adc_T >>= 4
        if adc_T & 0x800:
            adc_T |= 0xF000

        # Initiate next reading
        self._i2c.writeto_mem(self._address, MPL3115A2_CTRL_REG1, bytes([0b00011010]))
        return adc_P, adc_T
