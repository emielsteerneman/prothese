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




# import matplotlib.pyplot as plt
# from collections import defaultdict
# import numpy as np
# import scipy.signal as signal

# # Bestandspad aanpassen
# bestandspad = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250312_162604.txt"

# x_values = []
# y_values = []

# # Bestand inlezen en verwerken
# with open(bestandspad, "r") as f:
#     for line in f:
#         links, *rechts = line.strip().split()  # Splits op spaties
#         rechts = " ".join(rechts)
#         # if len(delen) == 2:  # Verwacht formaat controleren
#         # _, data = delen
#         getallen = rechts.split(",")  # Splits de laatste twee getallen
#         if len(getallen) >= 3:  # Controleer of er genoeg data is
#             try:
#                 x = float(getallen[0]) 
#                 y = float(getallen[2])  
                
#                 x_values.append(x)  # ✅ Append values inside the loop
#                 y_values.append(y)

#             except ValueError:
#                 print(f"Fout bij verwerken van regel: {line}")


# # y_values = list(np.diff(y_values))
# # y_values = [0] + y_values

# if len(y_values) > 10:  # ✅ Ensure there's enough data for filtering
#     fs = 1000  # Sample rate in Hz (adjust if needed)
#     f0 = 50    # Frequency to remove (50Hz noise)
#     Q = 30     # Quality factor (adjust for sharpness of notch)

#     # Create notch filter coefficients
#     b, a = signal.iirnotch(f0, Q, fs)

#     # Apply notch filter to y-values
#     filtered_y_values = signal.filtfilt(b, a, y_values)
    
#     # 📈 **Plot Original vs Filtered Data**
#     plt.figure(figsize=(10, 5))
#     plt.plot(x_values, y_values, marker="o", linestyle="-", label="Original Data", alpha=0.5)
#     # plt.plot(x_values, filtered_y_values, marker="o", linestyle="-", label="Filtered (50Hz Notch)", color="red")
#     plt.xlabel("Timestamp")
#     plt.ylabel("Input")
#     plt.title("Encoder Angular Velocity (degrees/s)")
#     plt.legend()
#     plt.grid(True)
#     plt.show()
# else:
#     print("Niet genoeg data om filtering toe te passen.")














# import matplotlib.pyplot as plt
# from collections import defaultdict

# # Bestandspad aanpassen
# bestandspad = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250303_10300tot11800_motorspeed_encoderspeed.txt"

# # Dictionary om x-waarden te groeperen en bijbehorende y-waarden op te slaan
# data_dict = defaultdict(list)

# # Bestand inlezen en verwerken
# with open(bestandspad, "r") as f:
#     for line in f:
#         delen = line.strip().split()  # Splits op spaties
#         if len(delen) == 2:  # Verwacht formaat controleren
#             _, data = delen
#             getallen = data.split(",")  # Splits de laatste twee getallen
#             if len(getallen) >= 2:  # Controleer of er genoeg data is
#                 try:
#                     x = float(getallen[-3])  # Twee-na-laatste getal (X-as)
#                     y = abs(float(getallen[-1]))  # Absolute waarde van het laatste getal (Y-as)
#                     data_dict[x].append(y)  # Opslaan in dictionary
#                 except ValueError:
#                     print(f"Fout bij verwerken van regel: {line}")

# # Nieuwe x- en y-waarden berekenen
# x_values = []
# y_values = []

# for x, y_list in data_dict.items():
#     if len(y_list) > 0:  # Voorkom deling door nul
#         gemiddelde_y = (sum(y_list) / len(y_list)) # Gemiddelde berekenen en door 2 delen
#         x_values.append(x)
#         y_values.append(y_list)

# # Sorteer de waarden op x-as voor een doorlopende lijn
# # sorted_pairs = sorted(zip(x_values, y_values))
# # x_values, y_values = zip(*sorted_pairs)  # Splits de gesorteerde paren

# # Data plotten als lijn
# plt.plot(x_values, y_values, marker="o", linestyle="-")  # Doorlopende lijn met punten
# plt.xlabel("X-waarde (een-na-laatste getal)")
# plt.ylabel("Gemiddelde van absolute Y-waarden gedeeld door 2")
# plt.title("Lijnplot met verwerkte gegevens")
# plt.grid(True)
# plt.show()
