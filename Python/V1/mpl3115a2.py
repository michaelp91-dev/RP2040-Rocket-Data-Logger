# mpl3115a2.py
from machine import I2C
from struct import unpack
from micropython import const

# Define constants for register addresses.
MPL3115A2_ADDRESS = const(0x60)
MPL3115A2_WHOAMI = const(0x0C) # WHO_AM_I register
MPL3115A2_ID = const(0xC4)     # Expected device ID
MPL3115A2_CTRL_REG1 = const(0x26)
MPL3115A2_PT_DATA_CFG = const(0x13)
MPL3115A2_STATUS = const(0x00)
MPL3115A2_OUT_P_MSB = const(0x01) # Start of pressure data
MPL3115A2_OUT_T_MSB = const(0x04) # Start of temperature data

class MPL3115A2:
    """Driver for the MPL3115A2 pressure/temperature sensor."""
    def __init__(self, i2c_bus):
        """
        Initializes the driver.
        :param i2c_bus: The shared I2C bus instance.
        """
        self._i2c = i2c_bus
        self._address = MPL3115A2_ADDRESS

    def _test_i2c_device(self):
        """Scans the I2C bus to see if a device at the address is present."""
        try:
            return self._address in self._i2c.scan()
        except OSError:
            return False

    def init(self):
        """
        Initializes the sensor hardware.
        Checks the device ID and sets up one-shot measurement mode.
        """
        if not self._test_i2c_device():
            print("MPL3115A2 I2C Fail!")
            return False
        
        # Read the WHO_AM_I register to get the device ID.
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
        # Set up for a one-shot measurement. This will be triggered in get_raw_data.
        self._i2c.writeto_mem(self._address, MPL3115A2_CTRL_REG1, bytes([0b00011010]))
        return True

    def get_raw_data(self):
        """
        Reads raw pressure and temperature data.
        :return: A tuple containing the raw (pressure, temperature) values.
        """
        # Point to the start of the data registers.
        self._i2c.writeto(self._address, bytes([MPL3115A2_OUT_P_MSB]), False)
        # Read all 5 data bytes at once (3 for pressure, 2 for temperature).
        data = self._i2c.readfrom(self._address, 5)

        # Reconstruct the 20-bit pressure value from 3 bytes.
        adc_P = (data[0] << 16) | (data[1] << 8) | data[2]
        adc_P >>= 4 # The value is left-justified, so shift right by 4.

        # Reconstruct the 12-bit temperature value from 2 bytes.
        adc_T = (data[3] << 8) | data[4]
        adc_T >>= 4 # Value is left-justified, shift right by 4.
        
        # Handle negative temperatures (12-bit two's complement).
        if adc_T & 0x800:
            adc_T |= 0xF000 # Manually sign-extend to a 16-bit negative number.

        # Trigger the next one-shot reading so it's ready for the next call.
        self._i2c.writeto_mem(self._address, MPL3115A2_CTRL_REG1, bytes([0b00011010]))
        
        return adc_P, adc_T
      
