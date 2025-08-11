# mpl3115a2.py
from machine import I2C
from struct import unpack
from micropython import const

# --- Constants (no changes)
MPL3115A2_ADDRESS = const(0x60)
MPL3115A2_WHOAMI = const(0x0C)
MPL3115A2_ID = const(0xC4)
MPL3115A2_CTRL_REG1 = const(0x26)
MPL3115A2_OUT_P_MSB = const(0x01)

class MPL3115A2:
    """Driver for the MPL3115A2 pressure/temperature sensor."""
    def __init__(self, i2c_bus):
        self._i2c = i2c_bus
        self._address = MPL3115A2_ADDRESS

    def init(self):
        """Initializes the sensor and returns True on success."""
        try:
            # Check device ID
            sensor_id = self._i2c.readfrom_mem(self._address, MPL3115A2_WHOAMI, 1)[0]
            if sensor_id != MPL3115A2_ID:
                print(f"MPL3115A2 Error: Incorrect device ID. Expected {MPL3115A2_ID:#04x}, got {sensor_id:#04x}")
                return False
        except OSError as e:
            print(f"MPL3115A2 Error: I2C communication failed during init: {e}")
            return False
            
        # Set to Altimeter mode with oversampling
        self._set_mode_altimeter()
        return True

    def _set_mode_altimeter(self):
        """Sets the device to Altimeter mode to start measurements."""
        try:
            # Altimeter mode, OSR=128, start one-shot measurement
            self._i2c.writeto_mem(self._address, MPL3115A2_CTRL_REG1, b'\xb9')
        except OSError as e:
            print(f"MPL3115A2 Error: Failed to set mode: {e}")

    def get_raw_data(self):
        """Reads raw pressure and temperature data."""
        try:
            # Read 5 bytes of data starting from the pressure MSB
            data = self._i2c.readfrom_mem(self._address, MPL3115A2_OUT_P_MSB, 5)
            
            # Restart measurement for the next read
            self._set_mode_altimeter()

            # Unpack pressure and temperature
            pressure_raw = (data[0] << 16 | data[1] << 8 | data[2]) >> 4
            temp_raw = (data[3] << 8 | data[4]) >> 4
            
            # Handle negative temperature
            if temp_raw & 0x800:
                temp_raw |= 0xF000

            return pressure_raw, temp_raw
        except OSError as e:
            print(f"MPL3115A2 Error: Failed to read data: {e}")
            return 0, 0 # Return neutral values on failure
