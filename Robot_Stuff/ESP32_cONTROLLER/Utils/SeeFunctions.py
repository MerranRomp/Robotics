from machine import Pin, I2C, ADC
from time import sleep

IRsensors = []
min_vals = [294, 239, 33, 39, 0]
max_vals = [1023, 1023, 785, 938, 1023]
threshold = 50




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



sensors = []  # Global list to hold ADC sensor objects
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

def read_normalized_values():
    raw = read_raw_values()
    normalized = []
    for val, min_v, max_v in zip(raw, min_vals, max_vals):
        norm = (val - min_v) / (max_v - min_v) * 100
        norm = max(0, min(100, int(norm)))  # Clamp between 0–100
        normalized.append(norm)
    return normalized

def read_binary_values():
    norm = read_normalized_values()
    return [1 if v > threshold else 0 for v in norm]


 ##---------------ENCODER----------------##







