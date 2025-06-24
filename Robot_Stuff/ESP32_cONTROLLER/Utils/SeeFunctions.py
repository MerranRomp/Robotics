from machine import Pin, I2C, ADC
from time import sleep, ticks_ms, ticks_diff
from VL53L0X import VL53L0X
import math

tof = None
_previous_distance = 2000

 ##----------------TOF SENSOR----------------##
#from VL53L0X import VL53L0X  # Import the VL53L0X library for the Time-of-Flight sensor

def setup_VL53L0X(i2c): # Function to setup the VL53L0X Time-of-Flight sensor
    global tof
    tof = VL53L0X(i2c)
    tof.start()
    return


def TOFdistance(alpha=0.3):
    global _previous_distance, tof
    if tof is None:
        raise RuntimeError("TOF sensor not initialized. Call setup_VL53L0X() first.")
    
    new_distance = (tof.read() - 80) # or `tof.read_range_continuous_millimeters()` if using a different library
    _previous_distance = alpha * new_distance + (1 - alpha) * _previous_distance
    return _previous_distance

 ##----------------IR GROUND SENSOR----------------##

# Globals
IRsensors = []  # ADC sensor objects
prev_inputs = [200] * 5
prev_outputs = [200] * 5
min_vals = [294, 239, 33, 39, 0]
max_vals = [1023, 1023, 785, 938, 1023]
threshold = 50        # Set based on testing

# Filter coefficients
alpha_lowpass = 0.5
alpha_highpass = 0.9

def setup_ir_sensors(*pins):
    global IRsensors
    IRsensors = []
    for pin in pins:
        adc = ADC(Pin(pin))
        adc.atten(ADC.ATTN_11DB)  # 0-3.3V range
        adc.width(ADC.WIDTH_10BIT)  # 0-1023 resolution
        IRsensors.append(adc)

def read_raw_values():
    return [sensor.read() for sensor in IRsensors]

def apply_lowpass_filter(raw_values):
    global prev_outputs
    if prev_outputs == [0] * 5:
        prev_outputs = raw_values[:]  # warm start

    filtered = []
    for i in range(5):
        filtered_val = alpha_lowpass * raw_values[i] + (1 - alpha_lowpass) * prev_outputs[i]
        prev_outputs[i] = filtered_val
        filtered.append(int(filtered_val))
    return filtered

def read_normalized_values():
    filtered = apply_lowpass_filter(read_raw_values())
    normalized = []
    for val, min_v, max_v in zip(filtered, min_vals, max_vals):
        norm = (val - min_v) / (max_v - min_v) * 100
        norm = max(0, min(100, int(norm)))  # Clamp between 0–100
        normalized.append(norm)
    return normalized

def read_binary_values():
    norm = read_normalized_values()
    print("Norm:     ", norm)
    return [1 if v > threshold else 0 for v in norm]

## ----------------BUTTON DEBOUNCE/Interrupt --------------##

last_press_time = 0
DEBOUNCE_MS = 200  # default debounce threshold in milliseconds

def setup_button(pin_number, callback, debounce_ms=200):
    global last_press_time, DEBOUNCE_MS
    DEBOUNCE_MS = debounce_ms
    pin = Pin(pin_number, Pin.IN, Pin.PULL_UP)
    pin.irq(trigger=Pin.IRQ_FALLING, handler=lambda p: _debounced_handler(p, callback))

def _debounced_handler(pin, callback):
    global last_press_time
    current_time = ticks_ms()
    if ticks_diff(current_time, last_press_time) > DEBOUNCE_MS:
        last_press_time = current_time
        callback(pin)