# adxl343.py
from machine import I2C
from struct import unpack
from micropython import const

# --- Constants (no changes)
ADXL345_DEFAULT_ADDRESS = const(0x53)
ADXL345_REG_DEVID = const(0x00)
ADXL345_DEVICE_ID = const(0xE5)
ADXL345_REG_POWER_CTL = const(0x2D)
ADXL345_REG_DATAX0 = const(0x32)
ADXL345_REG_INT_ENABLE = const(0x2E)
ADXL345_REG_INT_SOURCE = const(0x30)

def _read_registers(i2c_bus, address, register, count):
    i2c_bus.writeto(address, bytes([register]), False)
    return i2c_bus.readfrom(address, count)

def _write_register(i2c_bus, address, register, value):
    i2c_bus.writeto(address, bytes([register, value]))

class ADXL343:
    """Driver for the ADXL343 3-axis accelerometer."""
    def __init__(self, i2c_bus, address=ADXL345_DEFAULT_ADDRESS):
        self._i2c = i2c_bus
        self._address = address
        self._enabled_interrupts = {}

    def init(self):
        """Initializes the sensor and returns True on success."""
        try:
            device_id = _read_registers(self._i2c, self._address, ADXL345_REG_DEVID, 1)[0]
            if device_id != ADXL345_DEVICE_ID:
                print(f"ADXL343 Error: Incorrect device ID. Expected {ADXL345_DEVICE_ID:#04x}, got {device_id:#04x}")
                return False
        except OSError as e:
            print(f"ADXL343 Error: I2C communication failed during init: {e}")
            return False
            
        _write_register(self._i2c, self._address, ADXL345_REG_POWER_CTL, 0x08)  # Enable measurement
        _write_register(self._i2c, self._address, ADXL345_REG_INT_ENABLE, 0x00)  # Disable all interrupts
        return True

    def get_raw_acceleration(self):
        """Reads and returns raw acceleration data (x, y, z)."""
        try:
            data = _read_registers(self._i2c, self._address, ADXL345_REG_DATAX0, 6)
            return unpack("<hhh", data)
        except OSError as e:
            print(f"ADXL343 Error: Failed to read acceleration data: {e}")
            return 0, 0, 0 # Return neutral values on failure

    def enable_motion_detection(self, *, threshold=18):
        """Enables the activity detection interrupt."""
        # AC mode, enable activity detection on all axes
        _write_register(self._i2c, self._address, 0x27, 0b01110000)
        _write_register(self._i2c, self._address, 0x24, threshold)  # Activity threshold
        _write_register(self._i2c, self._address, 0x2E, 0x10)  # Enable 'Activity' interrupt
        self._enabled_interrupts["motion"] = True

    def motion_detected(self):
        """Checks the interrupt source register to see if motion has occurred."""
        if "motion" in self._enabled_interrupts:
            try:
                # Reading this register clears the interrupt bits
                interrupt_source = _read_registers(self._i2c, self._address, ADXL345_REG_INT_SOURCE, 1)[0]
                return (interrupt_source & 0x10) > 0
            except OSError as e:
                print(f"ADXL343 Error: Failed to read interrupt source: {e}")
                return False
        return False
                                                   
