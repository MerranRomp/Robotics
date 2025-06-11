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

