# adxl343.py
from machine import I2C
from struct import unpack
from micropython import const

ADXL345_DEFAULT_ADDRESS = const(0x53)
ADXL345_REG_DEVID = const(0x00)
ADXL345_DEVICE_ID = const(0xE5)
ADXL345_REG_POWER_CTL = const(0x2D)
ADXL345_REG_DATAX0 = const(0x32)
ADXL345_REG_DATA_FORMAT = const(0x31)
ADXL345_REG_INT_ENABLE = const(0x2E)
ADXL345_REG_INT_SOURCE = const(0x30)

def _read_registers(i2c_bus, address, register, count):
    i2c_bus.writeto(address, bytes([register]), False)
    return i2c_bus.readfrom(address, count)

def _write_register(i2c_bus, address, register, value):
    i2c_bus.writeto(address, bytes([register, value]))

class ADXL343:
    """Driver for the ADXL345 3-axis accelerometer using shared I2C."""
    def __init__(self, i2c_bus, address=ADXL345_DEFAULT_ADDRESS):
        self._i2c = i2c_bus
        self._address = address
        self._enabled_interrupts = {}

    def init(self):
        device_id = _read_registers(self._i2c, self._address, ADXL345_REG_DEVID, 1)[0]
        if device_id != ADXL345_DEVICE_ID:
            raise OSError(f"ADXL345 not found! Expected ID: 0x{ADXL345_DEVICE_ID:02X}, Got: 0x{device_id:02X}")
        _write_register(self._i2c, self._address, ADXL345_REG_POWER_CTL, 0x08)  # Enable measurement
        _write_register(self._i2c, self._address, ADXL345_REG_INT_ENABLE, 0x00)  # Disable interrupts initially
        print("ADXL345 OK!")
        return True

    def get_raw_acceleration(self):
        """Reads raw acceleration data."""
        data = _read_registers(self._i2c, self._address, ADXL345_REG_DATAX0, 6)
        x, y, z = unpack("<hhh", data)
        return x, y, z

    def enable_motion_detection(self, *, threshold=18):
        """
        The activity detection parameters.

        :param int threshold: The value that acceleration on any axis must exceed to\
        register as active. The scale factor is 62.5 mg/LSB.
        """
        _write_register(self._i2c, self._address, 0x27, 0b01110000)  # Enable activity on X, Y, Z
        _write_register(self._i2c, self._address, 0x24, threshold)  # Set threshold
        _write_register(self._i2c, self._address, 0x2E, 0x10)  # Enable activity interrupt
        self._enabled_interrupts["motion"] = True

    def disable_motion_detection(self):
        """Disable motion detection."""
        current_int_enable = _read_registers(self._i2c, self._address, 0x2E, 1)[0]
        _write_register(self._i2c, self._address, 0x2E, current_int_enable & ~0x10)
        self._enabled_interrupts.pop("motion", None)

    def motion_detected(self):
        """Returns True if motion is detected since the last check."""
        if "motion" in self._enabled_interrupts:
            interrupt_source_register = _read_registers(self._i2c, self._address, ADXL345_REG_INT_SOURCE, 1)[0]
            return (interrupt_source_register & 0x10) > 0
        return False

    @property
    def events(self):
        """
        Returns a dictionary indicating which enabled events have occurred.
        Possible keys: "motion". Value is True if the event occurred, False otherwise.
        """
        interrupt_source_register = _read_registers(self._i2c, self._address, ADXL345_REG_INT_SOURCE, 1)[0]
        event_status = {}
        if "motion" in self._enabled_interrupts:
            event_status["motion"] = (interrupt_source_register & 0x10) > 0
        return event_status
