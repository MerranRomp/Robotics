from machine import Pin, I2C, ADC
from time import sleep


 ##----------------TOF SENSOR----------------##
from VL53L0X import VL53L0X  # Import the VL53L0X library for the Time-of-Flight sensor

def setup_VL53L0X(): # Function to setup the VL53L0X Time-of-Flight sensor
    i2c = I2C(0, scl=Pin(22), sda=Pin(21))  # adjust pins as needed
    tof = VL53L0X(i2c)
    tof.start()
    return tof


def TOF_read_distance():
    return tof.read() # Read distance from the TOF sensor in mm

def filtered_distance(tof, previous, alpha=0.3):# Function to filter the distance reading from the TOF sensor
    new = TOF_read_distance(tof)
    return alpha * new + (1 - alpha) * previous




 ##----------------IR GROUND SENSOR----------------##


def setup_ir_sensors():
    # Define the analog pin numbers (adjust as needed for your board)
    sensor_pins = [32, 33, 34, 35, 2]  # ADC-capable pins on ESP32
    sensors = [ADC(Pin(pin)) for pin in sensor_pins]

    adc_pins = [ADC(Pin(pin)) for pin in sensor_pins]
    for adc in adc_pins:
        adc.atten(ADC.ATTN_11DB)  # Full 0-3.3V range (adjust as needed)

    return sensors

# Calibration values
min_vals = [294, 239, 33, 39, 0]
max_vals = [1023, 1023, 785, 938, 1023]


def read_sensors():
    """Read raw values from line sensors (0–4095)."""
    return [adc.read() for adc in adc_pins]

def normalize(val, i):
    if max_vals[i] == min_vals[i]:
        return 0
    return int(100 * (val - min_vals[i]) / (max_vals[i] - min_vals[i]))

def get_normalized(sensor_vals):
    return [normalize(reading, i) for i, reading in enumerate(sensor_vals)]

def get_binary(normalized_vals, threshold=50):
    return [1 if v < threshold else 0 for v in normalized_vals]

def get_inverse(normalized_vals):
    return [100 - v for v in normalized_vals]



 ##--------------------------------##

