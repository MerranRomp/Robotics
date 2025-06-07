#Code to replicate the measurments of the VL53L0X distance sensor
#SCL: Pin 22
#SDA: Pin 21


from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep
from VL53L0X import VL53L0X

# Setup I2C and VL53L0X
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
tof = VL53L0X(i2c)
tof.set_measurement_timing_budget(20000)  # 20ms = ~50Hz
tof.start()

# Data collection
data = []
start_time = ticks_ms()

print("Sampling for 10 seconds...")

while ticks_diff(ticks_ms(), start_time) < 10_000:
    d = tof.read()
    t = ticks_diff(ticks_ms(), start_time)
    data.append((t, d))  # Store (time, distance) tuple

print("Done sampling, saving...")

# Save to text file
with open("distance_log.txt", "w") as f:
    for t, d in data:
        f.write(f"{t},{d}\n")

print("Saved to distance_log.txt")

