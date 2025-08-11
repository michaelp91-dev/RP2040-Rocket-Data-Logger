# main.py
from machine import Pin, I2C
import time
import os
from mpl3115a2 import MPL3115A2
from adxl343 import ADXL343

# --- Centralized Configuration ---
CONFIG = {
    # Hardware Pins
    "led_pin": 11,
    "i2c_bus_id": 1,
    "i2c_scl_pin": 3,
    "i2c_sda_pin": 2,
    "i2c_freq": 400000,
    # Behavior
    "standby_duration_s": 10,
    "logging_duration_ms": 10000, # 10 seconds
    "log_interval_ms": 10, # Log at 100 Hz
    "motion_threshold": 50, # Sensitivity for motion detection
    # File System
    "data_dir": "flight_data",
}

class FlightComputer:
    """Encapsulates all flight computer logic and state."""
    
    # Define system states
    STATE_BOOT = "BOOT"
    STATE_INIT_FAIL = "INIT_FAIL"
    STATE_STANDBY = "STANDBY"
    STATE_ARMED = "ARMED"
    STATE_LOGGING = "LOGGING"
    STATE_POST_FLIGHT = "POST_FLIGHT"
    STATE_FAULT = "FAULT"

    def __init__(self, config):
        self.config = config
        self.state = self.STATE_BOOT
        self._state_enter_time = time.ticks_ms()
        
        # Hardware
        self.led = Pin(self.config["led_pin"], Pin.OUT)
        self.i2c = I2C(self.config["i2c_bus_id"], scl=Pin(self.config["i2c_scl_pin"]), sda=Pin(self.config["i2c_sda_pin"]), freq=self.config["i2c_freq"])
        self.altimeter = MPL3115A2(self.i2c)
        self.accelerometer = ADXL343(self.i2c)
        
        # Data logging
        self.log_file = None
        self.log_filename = ""
        self.next_log_time = 0

    def run(self):
        """Main execution loop."""
        print(f"[{self.state}] Power on.")
        self._initialize_sensors()

        while True:
            self.manage_state()
            self.update_led()
            time.sleep_ms(10) # Main loop delay

    def _change_state(self, new_state):
        """Changes the system state and logs the transition."""
        if self.state != new_state:
            print(f"[{self.state}] -> [{new_state}]")
            self.state = new_state
            self._state_enter_time = time.ticks_ms()

    def _initialize_sensors(self):
        """Attempts to initialize all sensors."""
        print(f"[{self.state}] Initializing sensors...")
        if not self.altimeter.init() or not self.accelerometer.init():
            self._change_state(self.STATE_INIT_FAIL)
        else:
            print(f"[{self.state}] Sensors initialized successfully!")
            self._change_state(self.STATE_STANDBY)

    def manage_state(self):
        """The core state machine logic."""
        
        # --- BOOT ---
        if self.state == self.STATE_BOOT:
            # This state is handled by __init__ and _initialize_sensors
            pass
        
        # --- STANDBY ---
        # Wait for a configured duration before arming
        elif self.state == self.STATE_STANDBY:
            elapsed_time = time.ticks_diff(time.ticks_ms(), self._state_enter_time)
            if elapsed_time > self.config["standby_duration_s"] * 1000:
                self.accelerometer.enable_motion_detection(threshold=self.config["motion_threshold"])
                self._change_state(self.STATE_ARMED)

        # --- ARMED ---
        # Wait for motion to be detected
        elif self.state == self.STATE_ARMED:
            if self.accelerometer.motion_detected():
                if self._prepare_log_file():
                    self._change_state(self.STATE_LOGGING)
                    self.next_log_time = time.ticks_ms()
                else:
                    self._change_state(self.STATE_FAULT) # Change to fault if file prep fails

        # --- LOGGING ---
        # Record data for a configured duration
        elif self.state == self.STATE_LOGGING:
            current_time = time.ticks_ms()
            if time.ticks_diff(current_time, self.next_log_time) >= 0:
                self._log_data(current_time)
                self.next_log_time = time.ticks_add(self.next_log_time, self.config["log_interval_ms"])
            
            if time.ticks_diff(current_time, self._state_enter_time) >= self.config["logging_duration_ms"]:
                self._close_log_file()
                self._change_state(self.STATE_POST_FLIGHT)
        
        # --- POST_FLIGHT, INIT_FAIL, FAULT ---
        # These are terminal states. The system will halt here.
        elif self.state in [self.STATE_POST_FLIGHT, self.STATE_INIT_FAIL, self.STATE_FAULT]:
            pass # Do nothing, effectively halting meaningful operation

    def update_led(self):
        """Non-blocking LED manager to indicate current state."""
        # Simple patterns:
        # FAULT: Solid ON
        # ARMED: Fast blink
        # LOGGING: Double blink
        # STANDBY: Slow "heartbeat" blink
        if self.state in [self.STATE_INIT_FAIL, self.STATE_FAULT]:
            self.led.on()
        elif self.state == self.STATE_ARMED:
            self.led.toggle() # Fast toggle
        elif self.state == self.STATE_LOGGING:
             # Creates a "buh-bump... buh-bump..." effect
            if time.ticks_diff(time.ticks_ms(), self._state_enter_time) % 500 < 100:
                self.led.on()
            else:
                self.led.off()
        elif self.state == self.STATE_STANDBY:
            # Creates a slow "heartbeat"
            if time.ticks_diff(time.ticks_ms(), self._state_enter_time) % 2000 < 150:
                self.led.on()
            else:
                self.led.off()
        else: # BOOT, POST_FLIGHT
            self.led.off()

    def _prepare_log_file(self):
        """Creates a new, uniquely named log file."""
        try:
            os.mkdir(self.config["data_dir"])
        except OSError:
            pass # Directory already exists

        file_number = 1
        while True:
            filename = f"{self.config['data_dir']}/flight_{file_number:03d}.csv"
            try:
                os.stat(filename)
                file_number += 1
            except OSError:
                self.log_filename = filename
                break
        
        try:
            self.log_file = open(self.log_filename, 'w')
            self.log_file.write("time_ms,pressure_raw,temp_raw,accel_x,accel_y,accel_z\n")
            print(f"[{self.state}] Logging to {self.log_filename}")
            return True
        except Exception as e:
            print(f"[{self.state}] Error: Failed to create log file: {e}")
            return False

    def _log_data(self, current_time):
        """Reads from sensors and writes to the log file."""
        try:
            p_raw, t_raw = self.altimeter.get_raw_data()
            ax, ay, az = self.accelerometer.get_raw_acceleration()
            
            elapsed_time = time.ticks_diff(current_time, self._state_enter_time)
            self.log_file.write(f"{elapsed_time},{p_raw},{t_raw},{ax},{ay},{az}\n")
        except Exception as e:
            print(f"[{self.state}] FAULT: Halting due to sensor/write error: {e}")
            self._change_state(self.STATE_FAULT)

    def _close_log_file(self):
        """Closes the log file."""
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            print(f"[{self.state}] Log file closed.")

# --- Program Entry Point ---
if __name__ == "__main__":
    computer = FlightComputer(CONFIG)
    computer.run()
      
