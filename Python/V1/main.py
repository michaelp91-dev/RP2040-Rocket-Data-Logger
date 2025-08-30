# main.py - Slower Sample Rate for Power Stability
from machine import Pin, I2C
import time
import os
from mpl3115a2 import MPL3115A2
from adxl343 import ADXL343

# Power-on delay for sensor stabilization
time.sleep(5)

# --- Flight Configuration ---
LOADING_DURATION_S = 60
LOG_DURATION_S = 30
TARGET_SAMPLE_RATE_HZ = 100   # MODIFIED: Reduced from 450 to 50 Hz
BUFFER_SIZE = 250
LAUNCH_G_FORCE_THRESHOLD = 1.5

# --- Constants ---
LSB_PER_G = 256
LAUNCH_THRESHOLD_RAW = int(LAUNCH_G_FORCE_THRESHOLD * LSB_PER_G)

# --- Hardware Configuration ---
i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)
led_pin = Pin("LED", Pin.OUT)

# --- Sensor Initialization ---
print("Initializing sensors...")
try:
    mpl = MPL3115A2(i2c)
    accelerometer = ADXL343(i2c)
    accelerometer.init()
    print("Sensors initialized successfully!")
except Exception as e:
    print(f"Sensor initialization failed: {e}")
    raise SystemExit

# Allow power to stabilize after sensor init before file access
time.sleep(0.5)

# --- File Setup ---
file_number = 1
while True:
    filename = (f"{file_number:03d}_flight_log.csv")
    try:
        with open(filename, 'r'): pass
        file_number += 1
    except OSError: break

# --- Loading Phase ---
print(f"Loading phase... waiting for {LOADING_DURATION_S} seconds before arming.")
end_time = time.ticks_add(time.ticks_ms(), LOADING_DURATION_S * 1000)
while time.ticks_diff(end_time, time.ticks_ms()) > 0:
    led_pin.toggle()
    time.sleep_ms(250)
led_pin.off()
print("Loading phase complete.")

# --- Pre-Flight Arming and Logging ---
print(f"Ready for launch. Data will be saved to {filename}")

with open(filename, 'w') as f:
    f.write("Time_ms,Pressure_Raw,Temperature_Raw,Accel_X_Raw,Accel_Y_Raw,Accel_Z_Raw\n")
    f.flush()
    data_buffer = []
    total_readings = 0
    log_duration_ms = LOG_DURATION_S * 1000
    time_interval_us = 1_000_000 // TARGET_SAMPLE_RATE_HZ

    # --- Pre-Flight: Wait for Launch ---
    led_pin.on()
    print(f"Armed! Waiting for G-force > {LAUNCH_G_FORCE_THRESHOLD}g on the vertical Y-axis...")
    while True:
        _, ay_raw, _ = accelerometer.get_raw_acceleration()
        if abs(ay_raw) > LAUNCH_THRESHOLD_RAW:
            led_pin.off()
            break # Exit the arming loop and start logging

    print("--- LAUNCH DETECTED! LOGGING ACTIVE ---")
    
    # --- Main Logging Flow ---
    start_time_ms = time.ticks_ms()
    next_sample_time_us = time.ticks_us()

    while time.ticks_diff(time.ticks_ms(), start_time_ms) < log_duration_ms:
        if time.ticks_diff(time.ticks_us(), next_sample_time_us) >= 0:
            pressure_raw, temp_raw = mpl.get_raw_data()
            ax_raw, ay_raw, az_raw = accelerometer.get_raw_acceleration()
            elapsed_ms = time.ticks_diff(time.ticks_ms(), start_time_ms)

            data_line = f"{elapsed_ms},{pressure_raw},{temp_raw},{ax_raw},{ay_raw},{az_raw}"
            data_buffer.append(data_line)
            total_readings += 1

            if len(data_buffer) >= BUFFER_SIZE:
                f.write('\n'.join(data_buffer) + '\n')
                f.flush()
                data_buffer.clear()

            next_sample_time_us = time.ticks_add(next_sample_time_us, time_interval_us)

    if data_buffer:
        f.write('\n'.join(data_buffer) + '\n')
        f.flush()

# --- Post-Flight ---
led_pin.on()
end_time_ms = time.ticks_ms()
actual_duration_s = time.ticks_diff(end_time_ms, start_time_ms) / 1000.0
actual_rate_hz = total_readings / actual_duration_s

print("--- LOGGING FINISHED ---")
print(f"Logged {total_readings} readings in {actual_duration_s:.2f} seconds.")
print(f"Achieved sample rate: {actual_rate_hz:.2f} Hz")
print(f"Data saved to {filename}")
