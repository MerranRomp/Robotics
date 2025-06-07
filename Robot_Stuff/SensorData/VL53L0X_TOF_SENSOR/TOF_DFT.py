import numpy as np
import matplotlib.pyplot as plt

# Load data
data = np.loadtxt("Robot_Stuff/SensorData/TOF_Sensor_Output_10Sec.txt", delimiter=",")
times = data[:, 0] / 1000  # convert ms to seconds
distances = data[:, 1]

# DFT
fft_vals = np.fft.fft(distances)
freqs = np.fft.fftfreq(len(distances), d=(times[1] - times[0]))

# Shift for centered zero frequency
fft_vals_shifted = np.fft.fftshift(fft_vals)
freqs_shifted = np.fft.fftshift(freqs)

# Convert to decibels (log scale)
amplitude_db = 20 * np.log10(np.abs(fft_vals_shifted) + 1e-6)  # add epsilon to avoid log(0)

# Plot two-sided FFT in dB
plt.plot(freqs_shifted, amplitude_db)
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude (dB)")
plt.title("Distance Sensor FFT (Two-Sided, Log Scale)")
plt.grid(True)
plt.show()
