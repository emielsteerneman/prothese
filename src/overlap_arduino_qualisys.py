### === OUD BESTAND, ZIE SYNCHRONIZE_DATA.PY === ###


import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import re
import pandas as pd


# === Instellingen ===
json_path = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\Alg1.json"
vinger_marker = "Index"
target_markers = ["Tar1", "Tar2", "Tar3"]
prothese_markers = ["Motor_top", "Enc"]
afstand_drempel_mm = 20.0
y_start_drempel_boven_min_mm = 33.0  # 1 cm boven minimum //33.0 voor Nullmeting Anna
framerate = 128.0  # Hz

# === JSON inladen ===
with open(json_path, "r") as f:
    data = json.load(f)

# === Markerdata extraheren ===
def get_marker_data(name):
    for marker in data["Markers"]:
        if marker["Name"] == name:
            return np.array(marker["Parts"][0]["Values"])[:, :]
    raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

finger_pos = get_marker_data(vinger_marker)
targets = {name: get_marker_data(name) for name in target_markers}
prothese = {name: get_marker_data(name) for name in prothese_markers}

# === Functie: detecteer reiken op basis van Y-hoogte én aanraking ===
def detect_reiken_op_basis_van_y(finger_pos, target_pos, afstand_drempel, y_drempel_boven_min, fps):
    afstanden = np.linalg.norm(finger_pos - target_pos, axis=1)
    binnen = afstanden < afstand_drempel
    raak_frames = np.where(np.diff(binnen.astype(int)) == 1)[0] + 1  # raakmomenten

    y_min = np.min(finger_pos[:, 1])
    y_drempel = y_min + y_drempel_boven_min

    pogingen = []
    laatste_start_tijd = -np.inf

    for raak_frame in raak_frames:
        # Zoek het laatste frame vóór de aanraking waar Y < drempel
        voor_moment = np.where(finger_pos[:raak_frame, 1] < y_drempel)[0]
        if len(voor_moment) == 0:
            continue  # geen geldig startmoment gevonden
        start_frame = voor_moment[-1]
        duur = (raak_frame - start_frame) / fps
        start_tijd = start_frame / fps
        raak_tijd = raak_frame / fps

        # Vermijd dubbele pogingen (minimaal 1s tussenpoging)
        if start_tijd - laatste_start_tijd >= 1.0:
            pogingen.append((start_frame, raak_frame, start_tijd, raak_tijd, duur))
            laatste_start_tijd = start_tijd

    return pogingen



# === Analyse uitvoeren ===
# print("\n📊 Verbeterde reiktaakanalyse (Y-hoogte als startcriterium):")
# for target_name, target_pos in targets.items():
#     pogingen = detect_reiken_op_basis_van_y(
#         finger_pos, target_pos, afstand_drempel_mm, y_start_drempel_boven_min_mm, framerate
#     )

#     print(f"\n🎯 Target: {target_name}")
#     if not pogingen:
#         print("  Geen pogingen gevonden.")
#     for i, (start, raak, t_start, t_raak, duur) in enumerate(pogingen, 1):
#         print(f"  ▸ Poging {i}:")
#         print(f"     Start: frame {start}, tijd {t_start:.3f} s")
#         print(f"     Raak : frame {raak}, tijd {t_raak:.3f} s")
#         print(f"     Duur : {duur:.3f} s")


### plotten van de prothese markers, Motor_top en Enc
# fig, ax = plt.subplots(figsize=(12, 6))
tijd = np.arange(prothese["Motor_top"].shape[0]) / framerate
# ax.plot(tijd, prothese["Motor_top"][:, 1], label='Motor_top Y', color='red')
# ax.plot(tijd, prothese["Enc"][:, 1], label='Encoder Y', color='green')
# ax.set_xlabel("Tijd (s)")
# ax.set_ylabel("Positie (mm)")
# ax.set_title("Motor Top en Encoder Y-positie over tijd")
# ax.grid(True)
# ax.legend(title="Motor Top / Encoder")
# plt.tight_layout()
# plt.show()

#find peaks in motor_top data when value larger than 980 and print the time of the peaks
peaks, _ = find_peaks(prothese["Motor_top"][:, 1], height=980)

t_peaks = tijd[peaks]
time_diff_peaks = np.diff(t_peaks)
time_between_reach = 6.0
rest_time = np.where(time_diff_peaks > time_between_reach)[0]
peak_after_rest = np.insert(peaks[rest_time + 1], 0, peaks[0])
peak_after_rest = peak_after_rest[:-1]

#print timestamps of the peaks
print("Timestamps van de pieken:")
for i, peak in enumerate(peak_after_rest):
    print(f"Piek {i + 1}: {tijd[peak]:.2f} s")

plt.plot(tijd, prothese["Motor_top"][:, 1])
plt.plot(tijd[peaks], prothese["Motor_top"][peaks, 1], "x")
plt.plot(
    tijd[peak_after_rest],
    prothese["Motor_top"][peak_after_rest, 1],
    "o", label="Eerste pieken na rust", color="green"
)
plt.xlabel("Tijd (s)")
plt.ylabel("Positie (mm)")
plt.title("Motor Top Peaks")
plt.grid(True)
plt.show()

print(f"peaks after rest: {peak_after_rest}, length: {len(peak_after_rest)}")


#######################################
arduino_file_1 = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\logs\\log_20250423_160148.txt"#log_20250423_160148.txt  AnnaZoet_Alg1.txt  log_20250423_160822.txt
arduino_file_2 = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\logs\\log_20250423_160822.txt"
arduino_file_3 = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\logs\\log_20250423_161005.txt"

arduino_file = [arduino_file_1, arduino_file_2, arduino_file_3]

name = "AnnaZoet"

peaks_selected_0 = True
peaks_selected_1 = True
peaks_selected_2 = True

peaks_selected = [peaks_selected_0, peaks_selected_1, peaks_selected_2]

def load_log_file(filepath):
    all_numbers = []
    lines = open(filepath, "r").readlines()
    # Remove any empty lines
    lines = [line for line in lines if line.strip() != ""]

    column_names = ["python_time"] + re.findall("[a-zA-Z_]+", lines[0])
    if len(column_names) == 1:
        raise ValueError("No column names found in the first line of the log file.")

    for line in lines:
        # Find everything in the line that is made up of a bunch of numbers and a .
        matches = re.findall("-?[0-9\.]+", line)
        numbers = [ float(m) for m in matches ]
        all_numbers.append(numbers)
    return column_names, np.array(all_numbers)


def get(data, column_name):
    return data[:, column_names.index(column_name)]


for i in range(len(arduino_file)):
    column_names, data[i] = load_log_file(arduino_file[i])
    print(f"Data from {arduino_file[i]} loaded with {len(data[i])} rows and {len(column_names)} columns.")

# column_names, data_1 = load_log_file(arduino_file_1)
# column_names, data_2 = load_log_file(arduino_file_2)
# column_names, data_3 = load_log_file(arduino_file_3)

# data = [data_1, data_2, data_3]

bluetooth_time = []
log_counter = []
PID_timestamp = []
delta_PID_timestamp = []
elbow_angle = []
reference_velocity = []
raw_velocity = []
average_velocity = []
PID_integral = []
PID_derivative = []
output_velocity = []
motor_speed_MPC = []
ax = []
ay = []
az = []
gx = []
gy = []
gz = []
omega_x = []
error_velocity = []
motor_speed = []
arduino_time = []

for d in range(len(arduino_file)):
    bluetooth_time.append(get(data[d], "python_time"))
    log_counter.append(get(data[d], "N"))
    PID_timestamp.append(get(data[d], "t"))
    delta_PID_timestamp.append(get(data[d], "dt"))
    elbow_angle.append(get(data[d], "ENC_DEG"))
    reference_velocity.append(get(data[d], "REF"))
    raw_velocity.append(get(data[d], "VEL_RAW"))
    average_velocity.append(get(data[d], "VEL_AVG"))
    PID_integral.append(get(data[d], "I"))
    PID_derivative.append(get(data[d], "D"))
    output_velocity.append(get(data[d], "VEL_OUT"))
    motor_speed_MPC.append(get(data[d], "MS_MPC"))
    ax.append(get(data[d], "ax"))
    ay.append(get(data[d], "ay"))
    az.append(get(data[d], "az"))
    gx.append(get(data[d], "gx"))
    gy.append(get(data[d], "gy"))
    gz.append(get(data[d], "gz"))
    omega_x.append(get(data[d], "OX"))
    error_velocity.append(reference_velocity[d] - average_velocity[d])
    motor_speed.append(output_velocity[d] + motor_speed_MPC[d])
    arduino_time.append((PID_timestamp[d] - PID_timestamp[d][0]) / 1e6)  # tijd in seconden




    # log_counter[d] = get(data[d], "N")
    # PID_timestamp[d] = get(data[d], "t")
    # delta_PID_timestamp[d] = get(data[d], "dt")
    # elbow_angle[d] = get(data[d], "ENC_DEG")
    # reference_velocity[d] = get(data[d], "REF")
    # raw_velocity[d] = get(data[d], "VEL_RAW")
    # average_velocity[d] = get(data[d], "VEL_AVG")
    # PID_integral[d] = get(data[d], "I")
    # PID_derivative[d] = get(data[d], "D")
    # output_velocity[d] = get(data[d], "VEL_OUT")
    # motor_speed_MPC[d] = get(data[d], "MS_MPC")
    # ax[d] = get(data[d], "ax")
    # ay[d] = get(data[d], "ay")
    # az[d] = get(data[d], "az")
    # gx[d] = get(data[d], "gx")
    # gy[d] = get(data[d], "gy")
    # gz[d] = get(data[d], "gz")
    # omega_x[d] = get(data[d], "OX")
    # error_velocity[d] = reference_velocity[d] - average_velocity[d]
    # motor_speed[d] = output_velocity[d] + motor_speed_MPC[d]
    # arduino_time[d] = (PID_timestamp[d] - PID_timestamp[d][0]) / 1e6  # tijd in seconden




# bluetooth_time_1 = get(data_1, "python_time")
# bluetooth_time_2 = get(data_2, "python_time")
# bluetooth_time_3 = get(data_3, "python_time")
# log_counter_1 = get(data_1, "N")
# log_counter_2 = get(data_2, "N")
# log_counter_3 = get(data_3, "N")
# PID_timestamp_1 = get(data_1, "t")
# PID_timestamp_2 = get(data_2, "t")
# PID_timestamp_3 = get(data_3, "t")
# delta_PID_timestamp_1 = get(data_1, "dt")
# delta_PID_timestamp_2 = get(data_2, "dt")
# delta_PID_timestamp_3 = get(data_3, "dt")
# elbow_angle_1 = get(data_1, "ENC_DEG")
# elbow_angle_2 = get(data_2, "ENC_DEG")
# elbow_angle_3 = get(data_3, "ENC_DEG")
# reference_velocity_1 = get(data_1, "REF")
# reference_velocity_2 = get(data_2, "REF")
# reference_velocity_3 = get(data_3, "REF")
# raw_velocity_1 = get(data_1, "VEL_RAW")
# raw_velocity_2 = get(data_2, "VEL_RAW")
# raw_velocity_3 = get(data_3, "VEL_RAW")
# average_velocity_1 = get(data_1, "VEL_AVG")
# average_velocity_2 = get(data_2, "VEL_AVG")
# average_velocity_3 = get(data_3, "VEL_AVG")
# PID_integral_1 = get(data_1, "I")
# PID_integral_2 = get(data_2, "I")
# PID_integral_3 = get(data_3, "I")
# PID_derivative_1 = get(data_1, "D")
# PID_derivative_2 = get(data_2, "D")
# PID_derivative_3 = get(data_3, "D")
# output_velocity_1 = get(data_1, "VEL_OUT")
# output_velocity_2 = get(data_2, "VEL_OUT")
# output_velocity_3 = get(data_3, "VEL_OUT")
# motor_speed_MPC_1 = get(data_1, "MS_MPC")
# motor_speed_MPC_2 = get(data_2, "MS_MPC")
# motor_speed_MPC_3 = get(data_3, "MS_MPC")
# ax_1 = get(data_1, "ax")
# ax_2 = get(data_2, "ax")
# ax_3 = get(data_3, "ax")
# ay_1 = get(data_1, "ay")
# ay_2 = get(data_2, "ay")
# ay_3 = get(data_3, "ay")
# az_1 = get(data_1, "az")
# az_2 = get(data_2, "az")
# az_3 = get(data_3, "az")
# gx_1 = get(data_1, "gx")
# gx_2 = get(data_2, "gx")
# gx_3 = get(data_3, "gx")
# gy_1 = get(data_1, "gy")
# gy_2 = get(data_2, "gy")
# gy_3 = get(data_3, "gy")
# gz_1 = get(data_1, "gz")
# gz_2 = get(data_2, "gz")
# gz_3 = get(data_3, "gz")
# omega_x_1 = get(data_1, "OX")
# omega_x_2 = get(data_2, "OX")
# omega_x_3 = get(data_3, "OX")

# error_velocity = reference_velocity-average_velocity
# motor_speed = output_velocity + motor_speed_MPC


# arduino_time_1 = (PID_timestamp_1 - PID_timestamp_1[0])/1e6  # tijd in seconden


## plot arduino grafiek waarbij je pieken kunt selecteren

coordinates_arduino_peaks = []
list_idx = 0  # Extra teller voor je lijsten


for p in range(len(peaks_selected)):
    if not peaks_selected[p]:
        coordinates_arduino_peaks.append([]) # Add new empty list
        fig,ax = plt.subplots()
        ax.plot(arduino_time[p], az[p], label="az", color="red")
        ax.set_title("Klik op de pieken om de coordinaten op te slaan")
        line, = ax.plot(arduino_time[p], az[p], picker=5)

        def onclick(event):
            if event.artist == line:
                mouse_event = event.mouseevent
                xdata = mouse_event.xdata
                ydata = mouse_event.ydata

                idx = np.argmin(np.abs(arduino_time[p] - xdata))
                # -1 meaning, add to the last list
                coordinates_arduino_peaks[-1].append((arduino_time[p][idx], az[p][idx]))
                ax.plot(arduino_time[p][idx], az[p][idx], 'ro')
                fig.canvas.draw()

        cid = fig.canvas.mpl_connect("pick_event", onclick)

        plt.show()

        # print("geselecteerde pieken:")
        # for i in range(len(coordinates_arduino_peaks[p])):
            # print(f"time: {coordinates_arduino_peaks[p][i][0]}, az: {coordinates_arduino_peaks[p][i][1]}")

        print("geselecteerde pieken:")
        for time, az_value in coordinates_arduino_peaks[list_idx]:
            print(f"  Piek: time {time} , az {az_value}")

        # Opslaan
        df = pd.DataFrame(coordinates_arduino_peaks[list_idx], columns=['time', 'az'])
        df.to_csv(f'{name}_pieken_{p}.csv', index=False)

        print(f"Opgeslagen als '{name}_pieken_{p}.csv'")

        list_idx += 1  # Vergeet niet de teller omhoog te doen!


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ========== BEREKENINGEN ==========
# (Alles wat je eerder had, gecorrigeerd)

# Inladen van piekbestanden
peak_file = [pd.read_csv(f"{name}_pieken_{i}.csv") for i in range(len(arduino_file))]

# Aantal pieken
number_of_peaks = np.array([len(pf) for pf in peak_file])

# Start index per bestand
new_starting_point = np.zeros(len(arduino_file), dtype=int)
for i in range(1, len(arduino_file)):
    new_starting_point[i] = new_starting_point[i-1] + int(number_of_peaks[i-1])

print(f"tijd: {tijd}, length: {len(tijd)}")
print(f"new starting point: {new_starting_point}, length: {len(new_starting_point)}")
print(f"peak after rest: {peak_after_rest}, length: {len(peak_after_rest)}")

# Starttijd Qualisys pieken
qualisys_start_time_peaks = np.zeros(len(arduino_file))
for i in range(len(arduino_file)):
    qualisys_start_time_peaks[i] = tijd[peak_after_rest][new_starting_point[i]]

# Tijdverschil Arduino-Qualisys
time_diff_peaks_qualisys_arduino = np.zeros(len(arduino_file))
for i in range(len(arduino_file)):
    time_diff_peaks_qualisys_arduino[i] = coordinates_arduino_peaks[i][0][0] - qualisys_start_time_peaks[i]

print("Tijdverschillen:", time_diff_peaks_qualisys_arduino)

# ========== PLOTTEN ==========

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ========== PLOTTEN MET AANGEPASTE QUALISYS DATA ==========

for i in range(len(arduino_file)):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # ---- Qualisys Data Plotten ----
    qualisys_start_idx = new_starting_point[i]
    if i < len(arduino_file) - 1:
        qualisys_end_idx = new_starting_point[i+1]
    else:
        qualisys_end_idx = len(peak_after_rest)  # laatste stuk als het het laatste bestand is

    # X-data = tijd[peak_after_rest]
    # Y-data = Motor_top Y-positie

    ax1.plot(
        tijd[peak_after_rest][qualisys_start_idx:qualisys_end_idx],
        prothese["Motor_top"][peak_after_rest][qualisys_start_idx:qualisys_end_idx, 1],
        label="Motor_top Y (Qualisys)",
        color="blue"
    )
    ax1.set_ylabel('Positie (mm)')
    ax1.set_title(f'Qualisys Motor_top Y - Bestand {i}')
    ax1.grid(True)
    ax1.legend()

    # ---- Arduino Data Plotten ----
    corrected_time = arduino_time[i] - time_diff_peaks_qualisys_arduino[i]

    ax2.plot(corrected_time, az[i], label='Arduino AZ', color='red')
    ax2.set_xlabel('Tijd (s)')
    ax2.set_ylabel('Versnelling AZ')
    ax2.set_title(f'Arduino data - Bestand {i}')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()




# peak_file = [None] * len(arduino_file)
# for i in range(len(arduino_file)):
#     peak_file[i] = pd.read_csv(f"{name}_pieken_{i}.csv")


# ## hoeveel pieken zitten er in elk bestand
# number_of_peaks = np.zeros(len(arduino_file))        
# for i in range(len(arduino_file)):
#     number_of_peaks[i] = len(peak_file[i])
#     print(f"Number of peaks in file {arduino_file[i]}: {number_of_peaks[i]}")

# # bepaal startpunt voor de qualisys data gebaseerd op aantal pieken in de arduino data
# # new_starting_point = np.zeros(len(arduino_file)-1)
# # for i in range(len(arduino_file)-1):
# #     new_starting_point[i] = number_of_peaks[i]+1

# new_starting_point = np.zeros(len(arduino_file)-1, dtype=int)  # maak int array direct
# for i in range(len(arduino_file)-1):
#     new_starting_point[i] = int(number_of_peaks[i]) + 1

# # in qualisys de starttijd van de pieken bepalen
# qualisys_start_time_peaks = [0, np.zeros(len(arduino_file))]
# for i in range(len(arduino_file)-1):
#     qualisys_start_time_peaks[i+1] = tijd[peak_after_rest][new_starting_point[i]]

# # arduino time - (arduino time peak min de qualisys time peak)
# time_diff_peaks_qualisys_arduino = np.zeros(len(arduino_file))
# for i in range(len(arduino_file)):
#     time_diff_peaks_qualisys_arduino[i] = arduino_time -(coordinates_arduino_peaks[i][1][0] - qualisys_start_time_peaks[i])

# print(f"{time_diff_peaks}")
# if not peaks_selected:
#     coordinates_arduino_peaks = []

#     fig, ax = plt.subplots()
#     ax.plot(arduino_time_1, az_1, label="az", color="red")
#     ax.set_title("Klik op de pieken om de coordinaten op te slaan")
#     line, = ax.plot(arduino_time_1, az_1, picker=5)


#     def onclick(event):
#         if event.artist == line:
#             mouse_event = event.mouseevent
#             xdata = mouse_event.xdata
#             ydata = mouse_event.ydata

#             idx = np.argmin(np.abs(arduino_time_1 - xdata))
#             coordinates_arduino_peaks.append((arduino_time_1[idx], az_1[idx]))
#             ax.plot(arduino_time_1[idx], az_1[idx], 'ro')
#             fig.canvas.draw()


#     cid = fig.canvas.mpl_connect("pick_event", onclick)

#     plt.show()

#     print("geselecteerde pieken:")
#     for i in range(len(coordinates_arduino_peaks)):
#         print(f"time: {coordinates_arduino_peaks[i][0]}, az: {coordinates_arduino_peaks[i][1]}")

#     # Opslaan
#     df = pd.DataFrame(coordinates_arduino_peaks, columns=['time', 'az'])
#     df.to_csv('AnnaZoet_pieken_1.csv', index=False)

#     print("Opgeslagen als 'AnnaZoet_pieken_1.csv'")








# time = (PID_timestamp - PID_timestamp[0])/1e6  # tijd in seconden

# az_peaks, _ = find_peaks(az, height=1.1)
# t_az_peaks = time[az_peaks]
# time_between_reach = 3.0
# az_rest_time = np.where(np.diff(t_az_peaks) > time_between_reach)[0]
# az_peak_after_rest = np.insert(az_peaks[az_rest_time + 1], 0, az_peaks[0])


# === Plotten ===
# plt.plot(time, elbow_angle, label='Elbow Angle', color='red')
# plt.plot(time, ax, label='ax', color='red')
# plt.plot(time, ay, label='ay', color='green')
# plt.plot(time, az, label='az', color='blue')
# plt.plot(t_az_peaks, az[az_peaks], "x", label="Pieken in az")
# plt.plot(
#     time[az_peak_after_rest],
#     az[az_peak_after_rest],
#     "o",
#     label="Eerste pieken na rust",
#     color="green"
# )
# plt.xlabel("Tijd (s)")
# plt.ylabel("Hoek (graden)")
# plt.title("Elbow Angle over tijd")
# plt.grid(True)
# plt.tight_layout()
# plt.legend()
# plt.show()






## subplot met 2 plots waarbij de bovenste plot de eerste piek tot piek laat zien van qualisys en de onderste plot de pieken van de arduino
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
axs[0].plot(tijd, prothese["Motor_top"][:, 1], label='Motor_top Y', color='red')
axs[0].plot(tijd[peaks], prothese["Motor_top"][peaks, 1], "x", label="Pieken in Motor_top Y")
axs[0].plot(
    tijd[peak_after_rest],
    prothese["Motor_top"][peak_after_rest, 1],
    "o",
    label="Eerste pieken na rust",
    color="green"
)
axs[0].set_xlabel("Tijd (s)")
axs[0].set_ylabel("Positie (mm)")
axs[0].set_title("Motor Top Peaks")
axs[0].grid(True)

axs[1].plot(arduino_time_1-17.971488, az_1, label='az', color='red')
#laad de piekdata uit AnnaZoet_pieken_1.csv
df = pd.read_csv('AnnaZoet_pieken_1.csv')
print(df)
axs[1].plot(df['time']-17.971488, df['az'], "o", label="Pieken in az", color ="green")
axs[1].set_xlabel("Tijd (s)")
axs[1].set_ylabel("az (m/s^2)")
axs[1].set_title("az Peaks")
axs[1].grid(True)
axs[1].legend()
plt.tight_layout()
plt.show()


