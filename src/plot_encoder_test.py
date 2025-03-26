import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
import scipy.signal as signal
import math
import re
import os

def find_latest_logfile(offset=0):
    log_folder = "D:\BMT\Master\Thesis\Arduino\prothese\logs"
    all_logfiles = os.listdir(log_folder)
    all_logfiles = [ f for f in all_logfiles if f.startswith("log_2025")]
    log_filename = sorted(all_logfiles, reverse=True)[offset]
    return os.path.join(log_folder, log_filename)

def load_log_file(filepath):
    all_numbers = []
    lines = open(filepath, "r").readlines()
    for line in lines:
        # Find everything in the line that is made up of a bunch of numbers and a .
        matches = re.findall("-?[0-9\.]+", line)
        numbers = [ float(m) for m in matches ]
        all_numbers.append(numbers)
    return np.array(all_numbers)


logfile = find_latest_logfile(0)
print(f"Now opening {logfile}")
values = load_log_file(logfile)

for window_size in range(6, 21):
    moving_average = np.convolve(values[:,4], np.ones(window_size)/window_size, mode='valid') + window_size
    plt.subplot(2, 1, 1)
    plt.plot(values[window_size-1:,1], moving_average, label=f"Moving Average {window_size}", alpha=1)
    plt.grid()
    plt.title("Window Size Comparison Encoder Velocity at 31400 steps/s")
    plt.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    plt.yticks([]) 
    plt.xlim(values[:,1].min(), values[:,1].max())
    




plt.subplot(2, 1, 2)
plt.plot(values[:,1], values[:,4], label="Raw")
plt.scatter(values[:,1], np.zeros(len(values)), s=5, label = "Datapoint") 
# plt.plot(values[:,1], values[:,5], label="Moving Average")
# plt.plot(values[:,1], values[:,9]/1000., label="Motor Speed")
plt.plot(values[1:,1], np.diff(values[:,3]), label="Encoder diff")
# plt.plot(values[1:,1], values[1:,2]/1000., label="dt")


plt.xlabel("Timestamp (microseconds)")
plt.ylabel("Angular velocity (degrees/s)")
plt.grid()


plt.legend(bbox_to_anchor=(1.04, 0.7), loc="upper left")
plt.xlim(values[:,1].min(), values[:,1].max())
plt.tight_layout()
plt.show()



def plot_fft(signal, sampling_rate):
    N = len(signal)  # Number of samples
    freq = np.fft.fftfreq(N, d=1/sampling_rate)  # Frequency bins
    fft_values = np.fft.fft(signal)  # Compute FFT
    
    # Plot only the positive half of the spectrum
    half_N = N // 2  
    plt.figure(figsize=(8, 4))
    plt.plot(freq[:half_N], np.abs(fft_values[:half_N]) / N, label='Magnitude Spectrum')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.title('FFT of the Signal')
    plt.grid()
    plt.legend()
    plt.show()

plot_fft(values[:, 4], 1000)


exit()



total = 0
row_count = 0
values = []

mean = total / row_count
variance = sum((x - mean) ** 2 for x in values) / (row_count - 1)  # Bessel's correction
std_dev = math.sqrt(variance)

print(std_dev)
