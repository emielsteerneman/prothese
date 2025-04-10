import numpy as np
import matplotlib.pyplot as plt

## AS5048B
bestandsnaam_1 = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_114839.txt" #total: 2353.21, length: 1219.00, Average error velocity: 1.93
# bestandsnaam_1 = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_120208.txt" #total: 2351.00, length: 1233.00, Average error velocity: 1.91
# bestandsnaam_1 = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_120936.txt" #total: 2373.54, length: 1227.00, Average error velocity: 1.93

## AS5600
bestandsnaam_2 = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_123433.txt" #total: 2373.71, length: 1237.00, Average error velocity: 1.92
# bestandsnaam_2 = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_124740.txt" #total: 1943.10, length: 1035.00, Average error velocity: 1.88
# bestandsnaam_2 = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_130427.txt" #total: 2342.69, length: 1225.00, Average error velocity: 1.91
# bestandsnaam_2 = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_141526.txt" #total: 2342.69, length: 1225.00, Average error velocity: 1.91
# bestandsnaam = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250401_144552.txt" 


log_counters = []
PID_timestamps = []
delta_PID_timestamps = []
reference_velocities = []
encoder_values = []
elbow_angles = []
raw_velocities = []
average_velocities = []
error_velocities = []
PID_integrals = []
PID_derivatives = []
motor_speeds = []

PID_timestamps_1 = []
raw_velocities_1 = []
error_velocities_1 = []
average_velocities_1 = []


PID_timestamps_2 = []
error_velocities_2 = []
raw_velocities_2 = []
average_velocities_2 = []

with open(bestandsnaam_1, "r") as file_1:
    lines_1 = file_1.readlines()
    for line_1 in lines_1:
        line_1 = line_1.strip()
        if "," in line_1:
            log_counter, PID_timestamp_1, delta_PID_timestamp, reference_velocity, encoder_value, elbow_angle, raw_velocity_1, average_velocity_1, error_velocity_1, PID_integral, PID_derivative, motor_speed = line_1.split(",")

            # log_counters.append(float(log_counter))
            PID_timestamps_1.append(int(PID_timestamp_1))
            # delta_PID_timestamps.append(int(delta_PID_timestamp))
            # reference_velocities.append(float(reference_velocity))
            # encoder_values.append(float(encoder_value))
            # elbow_angles.append(float(elbow_angle))
            raw_velocities_1.append(float(raw_velocity_1))
            average_velocities_1.append(float(average_velocity_1))
            error_velocities_1.append(float(error_velocity_1))
            # PID_integrals.append(float(PID_integral))
            # PID_derivatives.append(float(PID_derivative))
            # motor_speeds.append(float(motor_speed))
            # PID_timestamp,
            # delta_PID_timestamp,
            # reference_velocity,
            # encoder_value,
            # elbow_angle,
            # raw_velocity,
            # average_velocity,
            # error_velocity,
            # PID_integral,
            # PID_derivative,
            # motor_speed

with open(bestandsnaam_2, "r") as file_2:
    lines_2 = file_2.readlines()
    for line_2 in lines_2:
        line_2 = line_2.strip()
        if "," in line_2:
            log_counter, PID_timestamp_2, delta_PID_timestamp, reference_velocity, encoder_value, elbow_angle, raw_velocity_2, average_velocity_2, error_velocity_2, PID_integral, PID_derivative, motor_speed = line_2.split(",")

            PID_timestamps_2.append(int(PID_timestamp_2))
            error_velocities_2.append(float(error_velocity_2))
            raw_velocities_2.append(float(raw_velocity_2))
            average_velocities_2.append(float(average_velocity_2))



# with open(bestandsnaam, "r") as file:
#     lines = file.readlines()
#     for line in lines:
#         line = line.strip()
#         if "," in line:
#             log_counter, PID_timestamp, encoder_value, elbow_angle, raw_velocity, average_velocity, error_velocity, motor_speed = line.split(",")

#             PID_timestamps.append(int(PID_timestamp))
#             # error_velocities.append(float(error_velocity))
#             raw_velocities.append(float(raw_velocity))
#             average_velocities.append(float(average_velocity))

# plt.plot(PID_timestamps, average_velocities, label="Encoder Oud")
# plt.xlabel("Tijd (s)")
# plt.ylabel("Snelheid (deg/s)")
# plt.title("Snelheidsfluctuaties")
# plt.legend()
# plt.show()

window_size = 12
raw_velocities_smooth_1 = np.convolve(raw_velocities_1, np.ones(window_size)/window_size, mode='valid')
raw_velocities_smooth_2 = np.convolve(raw_velocities_2, np.ones(window_size)/window_size, mode='valid')
PID_timestamps_smooth_1 = PID_timestamps_1[:len(raw_velocities_smooth_1)]
PID_timestamps_smooth_2 = PID_timestamps_2[:len(raw_velocities_smooth_2)]

# total error of the absolute error velocities
total_error_1 = sum(abs(v) for v in error_velocities_1)
length_1 = len(error_velocities_1)
average_error_1 = total_error_1 / len(error_velocities_1)
print(f"1: total: {total_error_1:.2f}, length: {length_1:.2f}, Average error velocity: {average_error_1:.2f}")

total_error_2 = sum(abs(v) for v in error_velocities_2)
length_2 = len(error_velocities_2)
average_error_2 = total_error_2 / len(error_velocities_2)
print(f"2: total: {total_error_2:.2f}, length: {length_2:.2f}, Average error velocity: {average_error_2:.2f}")



plt.figure(figsize=(10,5))
plt.plot(PID_timestamps_1, raw_velocities_1, label="Encoder Nieuw")
plt.plot(PID_timestamps_2, raw_velocities_2, label="Encoder Oud")
plt.xlabel("Tijd")
plt.ylabel("Snelheid (deg/s)")
plt.title("Snelheidsfluctuaties Raw")
plt.legend()
plt.grid()
plt.show()



plt.figure(figsize=(10,5))
plt.plot(PID_timestamps_smooth_1, raw_velocities_smooth_1, label="Encoder Nieuw")
plt.plot(PID_timestamps_smooth_2, raw_velocities_smooth_2, label="Encoder Oud")
plt.xlabel("Tijd")
plt.ylabel("Snelheid (deg/s)")
plt.title("Snelheidsfluctuaties Smooth")
plt.legend()
plt.grid()
plt.show()


from scipy.fftpack import fft

# FFT uitvoeren op de snelheidsmeting van encoder 1
fft_spectrum1 = np.abs(fft(raw_velocities_1))
fft_spectrum2 = np.abs(fft(raw_velocities_2))
frequenties_1 = np.fft.fftfreq(len(PID_timestamps_1), d=(PID_timestamps_1[1] - PID_timestamps_1[0]))  # Frequenties in Hz
frequenties_2 = np.fft.fftfreq(len(PID_timestamps_2), d=(PID_timestamps_2[1] - PID_timestamps_2[0]))  

# Zet tijdstempels om van microseconden (µs) naar seconden (s)
PID_timestamps_1 = np.array(PID_timestamps_1) / 1e6  
PID_timestamps_2 = np.array(PID_timestamps_2) / 1e6  

# Bereken de tijd tussen opeenvolgende samples
dt_1 = np.diff(PID_timestamps_1)  # Tijdstappen voor encoder 1 (in seconden)
dt_2 = np.diff(PID_timestamps_2)  # Tijdstappen voor encoder 2 (in seconden)

# Bereken de gemiddelde sampling rate in Hz
sample_rate_1 = 1 / np.mean(dt_1)
sample_rate_2 = 1 / np.mean(dt_2)

print(f"Sample rate Encoder 1: {sample_rate_1:.2f} Hz")
print(f"Sample rate Encoder 2: {sample_rate_2:.2f} Hz")


plt.figure(figsize=(10,5))
plt.plot(frequenties_1[:len(PID_timestamps_1)//2], fft_spectrum1[:len(PID_timestamps_1)//2], label="Encoder Nieuw")
plt.plot(frequenties_2[:len(PID_timestamps_2)//2], fft_spectrum2[:len(PID_timestamps_2)//2], label="Encoder Oud")
plt.xlabel("Frequentie (Hz)")
plt.ylabel("Amplitude")
plt.title("FFT - Frequentiespectrum van snelheid")
plt.legend()
plt.show()


#histogram

# Verwachte snelheid (stel deze in op jouw gewenste snelheid)
desired_speed = 12.5  # Bijvoorbeeld 50 RPM
from scipy.signal import butter, filtfilt

def lowpass_filter(data, cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

# Pas een low-pass filter toe om ruis te verwijderen
filtered_vel_1 = lowpass_filter(raw_velocities_1, cutoff=10, fs=sample_rate_1)
filtered_vel_2 = lowpass_filter(raw_velocities_2, cutoff=10, fs=sample_rate_2)

# Bereken de fout opnieuw met gefilterde data
filtered_error_1 = np.array(filtered_vel_1) - desired_speed
filtered_error_2 = np.array(filtered_vel_2) - desired_speed

# Plot het histogram opnieuw
plt.figure(figsize=(10,5))
plt.hist(filtered_error_1, bins=30, alpha=0.5, label="Encoder Nieuw (gefilterd)", color='blue')
plt.hist(filtered_error_2, bins=30, alpha=0.5, label="Encoder Oud (gefilterd)", color='red')
plt.xlabel("Snelheidsfout (RPM)")
plt.ylabel("Frequentie")
plt.title("Histogram van snelheidsfout na filtering")
plt.legend()
plt.show()


plt.figure(figsize=(10,5))
plt.plot(PID_timestamps_1, raw_velocities_1, label="Encoder Nieuw - Ruw signaal", alpha=0.5)
plt.plot(PID_timestamps_1, filtered_error_1, label="Encoder Nieuw - Gefilterd", color="red")
plt.xlabel("Tijd (s)")
plt.ylabel("Gemeten snelheid (RPM)")
plt.title("Effect van low-pass filtering op Encoder A")
plt.legend()
plt.show()



from scipy.signal import butter, sosfiltfilt

# Function for zero-phase low-pass filtering
def zero_phase_lowpass(data, cutoff, fs, order=3):
    nyq = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyq
    sos = butter(order, normal_cutoff, btype='low', analog=False, output='sos')
    return sosfiltfilt(sos, data)  # Zero-phase filtering

# Moving Average function
def moving_average(data, window_size=30):
    return np.convolve(data, np.ones(window_size)/window_size, mode='same')

# Low-pass filter settings
cutoff = 2  # Cutoff frequency in Hz
order = 3  # Filter order
fs = 46.35  # Replace with your actual sample rate (Hz)

# Apply the low-pass filter
filtered_vel_A = zero_phase_lowpass(raw_velocities_1, cutoff, fs, order)

# Correct for attenuation (rescale standard deviation)
filtered_vel_A *= (np.std(raw_velocities_1) / np.std(filtered_vel_A))

# Correct for DC offset (shift back to original mean)
offset = np.mean(raw_velocities_1) - np.mean(filtered_vel_A)
filtered_vel_A += offset

# Apply moving average filter (window size = 15)
smoothed_vel_A = moving_average(raw_velocities_1, window_size=15)

# Plot everything
plt.figure(figsize=(10,5))
plt.plot(PID_timestamps_1, raw_velocities_1, label="Encoder Nieuw - Raw", alpha=0.5)
plt.plot(PID_timestamps_1, filtered_vel_A, label="Encoder Nieuw - Low-Pass Filtered", color="red")
plt.plot(PID_timestamps_1, smoothed_vel_A, label="Encoder Nieuw - Moving Average", color="green", linestyle="dashed")
plt.xlabel("Time (s)")
plt.ylabel("Speed (RPM)")
plt.title("Raw vs. Filtered vs. Moving Average Speed Data")
plt.legend()
plt.show()


