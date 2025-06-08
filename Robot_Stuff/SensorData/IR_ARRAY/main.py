#this code is for ESP32 using MicroPython
# IR Array Calibration Script
#this script measures 100 times the values of the IR sensors
# and calculates the minimum and maximum values for calibration

#result for our IR array sensors were:
#min_vals = [294, 239, 33, 39, 0]
#max_vals = [1023, 1023, 785, 938, 1023]


from machine import ADC, Pin
from time import sleep
import sys  # For clean exit

# Define the analog pin numbers (adjust as needed for your board)
sensor_pins = [32, 33, 34, 35, 36]  # ADC-capable pins on ESP32
sensors = [ADC(Pin(pin)) for pin in sensor_pins]

# Configure ADC resolution and range (ESP32)
for s in sensors:
    s.atten(ADC.ATTN_11DB)       # 0–3.3V range
    s.width(ADC.WIDTH_10BIT)     # 0–1023 resolution

# Calibration data
min_vals = [1023] * 5
max_vals = [0] * 5

print("📏 Starting calibration — move sensors over WHITE and BLACK areas...")
sleep(2)

# Read 100 samples to determine min/max values
for _ in range(100):
    readings = [s.read() for s in sensors]
    for i in range(5):
        min_vals[i] = min(min_vals[i], readings[i])
        max_vals[i] = max(max_vals[i], readings[i])
    sleep(0.05)

# Print the result of calibration
print("\n✅ Calibration complete!")
print("Min values:", min_vals)
print("Max values:", max_vals)

# Exit the program after calibration
sys.exit()


