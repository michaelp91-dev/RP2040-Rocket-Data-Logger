# adxl343.py
from machine import I2C
from struct import unpack
from micropython import const

# Define constants for register addresses to make the code more readable.
ADXL345_DEFAULT_ADDRESS = const(0x53)
ADXL345_REG_DEVID = const(0x00)
ADXL345_DEVICE_ID = const(0xE5)
ADXL345_REG_POWER_CTL = const(0x2D)
ADXL345_REG_DATAX0 = const(0x32)
ADXL345_REG_DATA_FORMAT = const(0x31)
ADXL345_REG_INT_ENABLE = const(0x2E)
ADXL345_REG_INT_SOURCE = const(0x30)

def _read_registers(i2c_bus, address, register, count):
    """Helper function to read from I2C registers."""
    i2c_bus.writeto(address, bytes([register]), False)
    return i2c_bus.readfrom(address, count)

def _write_register(i2c_bus, address, register, value):
    """Helper function to write to an I2C register."""
    i2c_bus.writeto(address, bytes([register, value]))

class ADXL343:
    """Driver for the ADXL345 3-axis accelerometer using shared I2C."""
    def __init__(self, i2c_bus, address=ADXL345_DEFAULT_ADDRESS):
        """
        Initializes the driver.
        :param i2c_bus: The shared I2C bus instance.
        :param address: The I2C address of the sensor.
        """
        self._i2c = i2c_bus
        self._address = address
        self._enabled_interrupts = {}

    def init(self):
        """
        Initializes the sensor hardware.
        Checks the device ID and enables measurement mode.
        """
        # Read the device ID to confirm we're talking to the right sensor.
        device_id = _read_registers(self._i2c, self._address, ADXL345_REG_DEVID, 1)[0]
        if device_id != ADXL345_DEVICE_ID:
            raise OSError(f"ADXL345 not found! Expected ID: 0x{ADXL345_DEVICE_ID:02X}, Got: 0x{device_id:02X}")
        
        # Write 0x08 to the POWER_CTL register to enable measurement mode.
        _write_register(self._i2c, self._address, ADXL345_REG_POWER_CTL, 0x08)
        # Ensure all interrupts are disabled by default.
        _write_register(self._i2c, self._address, ADXL345_REG_INT_ENABLE, 0x00)
        print("ADXL345 OK!")
        return True

    def get_raw_acceleration(self):
        """
        Reads raw acceleration data for all three axes.
        :return: A tuple containing the raw (x, y, z) acceleration values.
        """
        # Read the 6 data registers (2 bytes per axis).
        data = _read_registers(self._i2c, self._address, ADXL345_REG_DATAX0, 6)
        # Unpack the 6 bytes into three 16-bit signed integers (little-endian).
        x, y, z = unpack("<hhh", data)
        return x, y, z

    def enable_motion_detection(self, *, threshold=18):
        """
        Enables the on-board 'activity' detection interrupt.
        :param int threshold: The acceleration value (in LSBs) that must be
        exceeded to trigger the interrupt. The scale factor is 62.5 mg/LSB.
        """
        # Configure activity detection for all axes, AC-coupled.
        _write_register(self._i2c, self._address, 0x27, 0b01110000)
        # Set the threshold for motion.
        _write_register(self._i2c, self._address, 0x24, threshold)
        # Enable the 'Activity' interrupt.
        _write_register(self._i2c, self._address, 0x2E, 0x10)
        self._enabled_interrupts["motion"] = True

    def disable_motion_detection(self):
        """Disables the 'activity' detection interrupt."""
        current_int_enable = _read_registers(self._i2c, self._address, 0x2E, 1)[0]
        # Clear only the activity interrupt bit without affecting other bits.
        _write_register(self._i2c, self._address, 0x2E, current_int_enable & ~0x10)
        self._enabled_interrupts.pop("motion", None)

    def motion_detected(self):
        """
        Checks if the 'activity' interrupt flag has been set.
        :return: True if motion was detected, False otherwise.
        """
        if "motion" in self._enabled_interrupts:
            # Read the interrupt source register.
            interrupt_source_register = _read_registers(self._i2c, self._address, ADXL345_REG_INT_SOURCE, 1)[0]
            # Check if the 'ACTIVITY' bit is high.
            return (interrupt_source_register & 0x10) > 0
        return False

    @property
    def events(self):
        """
        A more advanced method to check the status of all enabled events.
        :return: A dictionary of event statuses.
        """
        interrupt_source_register = _read_registers(self._i2c, self._address, ADXL345_REG_INT_SOURCE, 1)[0]
        event_status = {}
        if "motion" in self._enabled_interrupts:
            event_status["motion"] = (interrupt_source_register & 0x10) > 0
        return event_status
      
