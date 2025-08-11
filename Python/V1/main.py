# main.py
"""
A simple, procedural flight data logger for MicroPython.

This script initializes an accelerometer and an altimeter, waits for a fixed
period, then arms a motion-detection trigger. Once motion is detected, it
logs sensor data to a uniquely named CSV file on the Pico's flash storage
for a set duration.

Hardware Used:
- Controller: Adafruit ItsyBitsy RP2040
- Altimeter/Temp: Adafruit MPL3115A2
- Accelerometer: Adafruit ADXL343
"""
from machine import Pin, I2C
import time
import os
from mpl3115a2 import MPL3115A2
from adxl343 import ADXL343

# --- Hardware Configuration ---
# Set up the onboard LED for status indication.
led_pin = Pin(11, Pin.OUT)

# --- Shared I2C Initialization ---
# Both sensors are on the same I2C bus.
i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)

# --- Sensor Initialization ---
# Create instances of the sensor driver classes.
mpl = MPL3115A2(i2c)
accelerometer = ADXL343(i2c)

def blink(num_blinks, duration_ms=200):
    """Blinks the LED a specified number of times."""
    for _ in range(num_blinks):
        led_pin.on()
        time.sleep_ms(duration_ms)
        led_pin.off()
        time.sleep_ms(duration_ms)

def loading_mode_visuals(duration_s=45):
    """A blocking function to provide visual feedback with increasing blink speed."""
    start_time = time.time()
    while time.time() - start_time < duration_s:
        elapsed_time = time.time() - start_time
        if elapsed_time < 15:
            led_pin.on()
            time.sleep(1)
            led_pin.off()
            time.sleep(1)
        elif elapsed_time < 30:
            led_pin.on()
            time.sleep(0.5)
            led_pin.off()
            time.sleep(0.5)
        else:
            led_pin.on()
            time.sleep(0.25)
            led_pin.off()
            time.sleep(0.25)

# --- Main Flow ---
print("Power on")

# Initialize both sensors and check for success.
print("Initializing sensors...")
mpl_initialized = mpl.init()
adxl_initialized = accelerometer.init()

if mpl_initialized and adxl_initialized:
    print("Sensors initialized successfully!")
else:
    print("Sensor initialization failed. Exiting.")
    exit() # Stop the script if sensors fail.

# Fixed delays to manage the pre-flight sequence.
print("Waiting for 10 seconds...")
time.sleep(10)
print("Waiting for 45 seconds (loading mode)...")
loading_mode_visuals(45)

# Arm the motion detection feature on the accelerometer.
print("Enabling motion detection (pre flight mode)...")
motion_threshold = 50
accelerometer.enable_motion_detection(threshold=motion_threshold)
print(f"Motion detection enabled with threshold: {motion_threshold}")
print("Waiting for motion to start logging (in flight mode)...")

# --- File Setup ---
# Create a directory for the data logs if it doesn't exist.
raw_dir = "raw_data"
try:
    os.mkdir(raw_dir)
except OSError:
    # Directory already exists, which is fine.
    pass

# Find the next available file number to avoid overwriting old data.
file_number = 1
while True:
    raw_filename = raw_dir + (f"/raw_data_{file_number:03d}.csv")
    try:
        # If opening the file succeeds, it exists. Try the next number.
        with open(raw_filename, 'r'):
            pass
        file_number += 1
    except OSError:
        # If opening fails, the file does not exist. We can use this name.
        break

# --- Motion Detection Loop ---
motion_detected = False
if not motion_detected:
    # Loop indefinitely until motion is detected.
    while not motion_detected:
        time.sleep(0.01) # Small delay to prevent running too fast.
        if accelerometer.motion_detected():
            print("Motion detected! Starting data logging...")
            motion_detected = True
            break # Exit the inner while loop.
        time.sleep(0.1) # Longer delay if no motion is found to reduce CPU load.

# --- Data Logging Loop ---
if motion_detected:
    print(f"Recording raw data to: {raw_filename}")

    # Set up timing for the logging loop.
    time_interval_ms = 10 # Log at 100 Hz (10 ms interval).
    end_time_ms = 10000   # Log for 10 seconds.
    start_time_ms = time.ticks_ms() # Use ticks_ms for rollover-safe timing.
    next_sample_time_ms = start_time_ms + time_interval_ms

    # Open the file in write mode.
    with open(raw_filename, 'w') as f:
        # Write the header row for the CSV file.
        f.write("Time,Pressure_Raw,Temperature_Raw,Accel_X_Raw,Accel_Y_Raw,Accel_Z_Raw\n")
        
        # Loop until the logging duration has passed.
        while time.ticks_diff(time.ticks_ms(), start_time_ms) < end_time_ms:
            current_time_ms = time.ticks_ms()
            # This non-blocking check ensures we log at a consistent rate.
            if time.ticks_diff(current_time_ms, next_sample_time_ms) >= 0:
                # Get data from both sensors.
                pressure_raw, temperature_raw = mpl.get_raw_data()
                accel_x_raw, accel_y_raw, accel_z_raw = accelerometer.get_raw_acceleration()
                
                # Calculate elapsed time and write the data row.
                elapsed_time_ms = time.ticks_diff(current_time_ms, start_time_ms)
                f.write(f"{elapsed_time_ms},{pressure_raw},{temperature_raw},{accel_x_raw},{accel_y_raw},{accel_z_raw}\n")
                
                # Schedule the next sample time.
                next_sample_time_ms = time.ticks_add(next_sample_time_ms, time_interval_ms)
            
            # A very short sleep to prevent the loop from hogging the CPU.
            time.sleep_us(10)

    print(f"\nRaw data recording finished.")
else:
    print("Motion not detected within the pre-flight phase. Skipping data logging.")
    
print("Power off")
