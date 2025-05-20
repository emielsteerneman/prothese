### === JUISTE BESTAND VOOR SYNCHRONISATIE QUALISYS EN ARDUINO === ###
import json
import numpy as np
import matplotlib.pyplot as plt
import re
import pandas as pd

name = "WouterVisser"
algorithm = "Alg2"

qualisys_peaks_selected_0 = True

arduino_peaks_selected_0 = True
# arduino_peaks_selected_1 = True 
# arduino_peaks_selected_2 = True
# arduino_peaks_selected_3 = True
# arduino_peaks_selected_4 = True
# arduino_peaks_selected_5 = True

### === Qualisys Settings === ###
# file
qualisys_file_0 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
# qualisys_file_0 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_{algorithm}.json"

framerate = 128.0  # Hz
# markers
vinger_marker = "Index"
target_markers = ["Tar1", "Tar2", "Tar3"]
prothese_markers = ["Motor_top", "Enc"]
# thresholds
afstand_drempel_mm = 20.0
# y_start_drempel_boven_min_mm = 33.0  # 1 cm boven minimum //33.0 voor Nullmeting Anna

qualisys_file = [qualisys_file_0]
qualisys_peaks_selected = [qualisys_peaks_selected_0]

### === Arduino Settings === ###
# files
# arduino_file_1 = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\logs\\log_20250423_160148.txt"#log_20250423_160148.txt  AnnaZoet_Alg1.txt  log_20250423_160822.txt
# arduino_file_2 = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\logs\\log_20250423_160822.txt"
# arduino_file_3 = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\logs\\log_20250423_161005.txt"

arduino_file_1 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_1.txt"
# arduino_file_2 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_2.txt"
# arduino_file_3 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_3.txt"
# arduino_file_4 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_4.txt"
# arduino_file_5 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_5.txt"
# arduino_file_6 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_6.txt"



arduino_file = [arduino_file_1]#, arduino_file_2, arduino_file_3, arduino_file_4, arduino_file_5]#, arduino_file_6]
arduino_peaks_selected = [arduino_peaks_selected_0]#, arduino_peaks_selected_1, arduino_peaks_selected_2, arduino_peaks_selected_3, arduino_peaks_selected_4]#, arduino_peaks_selected_5]
data = [None] * len(arduino_file)


### === Functions === ###

## load arduino file
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

## get arduino data with column names from log
def get(data, column_name):
    return data[:, column_names.index(column_name)]


def get_marker_data(marker_name, qualisys_file):
    for marker in qualisys_file["Markers"]:
        if marker["Name"] == marker_name:
            return np.array(marker["Parts"][0]["Values"])[:, :]
    raise ValueError(f"Marker '{marker_name}' niet gevonden in JSON.")


### === Process Qualisys Data === ###
# laad qualisys data in
for i in range(len(qualisys_file)):
    with open(qualisys_file[i], "r") as f:
        qualisys_file[i] = json.load(f)

# extraheer marker data
finger_pos = [None] * len(qualisys_file)
targets = [None] * len(qualisys_file)
prothese = [None] * len(qualisys_file)
qualisys_time = [None] * len(qualisys_file)

for i in range(len(qualisys_file)):
    finger_pos[i] = get_marker_data(vinger_marker, qualisys_file[i])
    targets[i] = {marker_name: get_marker_data(marker_name, qualisys_file[i]) for marker_name in target_markers}
    prothese[i] = {marker_name: get_marker_data(marker_name, qualisys_file[i]) for marker_name in prothese_markers}
    qualisys_time[i] = np.arange(prothese[i]["Motor_top"].shape[0]) / framerate

###### beginpunt van de beweging bepalen, voor het syncen met arduino. Het beginpunt van de reikpoging bepalen om duur van de reikbeweging te bepalen.
###### 1e is op basis van schouderophaal, 2e is op basis van threshold y-hoogte Index


### bepaal beginpunt van de beweging op basis van schouderophaal door op pieken te klikken

for i in range(len(qualisys_peaks_selected)):
    print(f"shape of {i}: {prothese[i]['Motor_top'].shape}")

coordinates_qualisys_peaks = []
list_idx_qualisys = 0  # Extra teller voor je lijsten
for i in range(len(qualisys_peaks_selected)):
    if not qualisys_peaks_selected[i]:
        coordinates_qualisys_peaks.append([]) # Add new empty list
        fig,ax = plt.subplots()
        ax.plot(qualisys_time[i], prothese[i]["Motor_top"][:,1], label="prosthesis upper arm", color="red")
        ax.set_title("Klik op de qualisys pieken van de schouderophaal om de coordinaten op te slaan")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Motor Top Position y-axis (mm)")
        line, = ax.plot(qualisys_time[i], prothese[i]["Motor_top"][:,1], picker=5)

        def onclick(event):
            if event.artist == line:
                mouse_event = event.mouseevent
                xdata = mouse_event.xdata
                ydata = mouse_event.ydata

                idx = np.argmin(np.abs(qualisys_time[i] - xdata))
                # -1 meaning, add to the last list
                coordinates_qualisys_peaks[-1].append((qualisys_time[i][idx], prothese[i]["Motor_top"][idx,1]))
                ax.plot(qualisys_time[i][idx], prothese[i]["Motor_top"][idx, 1], 'ro')
                fig.canvas.draw()

        cid = fig.canvas.mpl_connect("pick_event", onclick)

        plt.show()

        print("geselecteerde pieken:")
        for qualisys_time, motor_top_value in coordinates_qualisys_peaks[list_idx_qualisys]:
            print(f"  Piek: time {qualisys_time} , motor_top {motor_top_value}")

        # Opslaan
        df = pd.DataFrame(coordinates_qualisys_peaks[list_idx_qualisys], columns=['time', 'motor_top'])
        df.to_csv(f'{name}_pieken_qualisys_{algorithm}_{i}.csv', index=False)

        print(f"Opgeslagen als '{name}_pieken_qualisys_{algorithm}_{i}.csv'")

        list_idx_qualisys += 1  # Vergeet niet de teller omhoog te doen!

## in coordinates_qualisys_peaks staan de pieken van de motor_top met tijd en hoogte van de piek, deze moeten nu gesyncroniseerd worden met de arduino data.




### === Process Arduino Data === ###

## load Arduino files
for i in range(len(arduino_file)):
    column_names, data[i] = load_log_file(arduino_file[i])
    print(f"Data from {arduino_file[i]} loaded with {len(data[i])} rows and {len(column_names)} columns.")


# Load the data from the Arduino log files
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
a_x = []
a_y = []
a_z = []
g_x = []
g_y = []
g_z = []
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
    a_x.append(get(data[d], "ax"))
    a_y.append(get(data[d], "ay"))
    a_z.append(get(data[d], "az"))
    g_x.append(get(data[d], "gx"))
    g_y.append(get(data[d], "gy"))
    g_z.append(get(data[d], "gz"))
    omega_x.append(get(data[d], "OX"))
    error_velocity.append(reference_velocity[d] - average_velocity[d])
    motor_speed.append(output_velocity[d] + motor_speed_MPC[d])
    arduino_time.append((PID_timestamp[d] - PID_timestamp[d][0]) / 1e6)  # tijd in seconden


## select peaks from az in Arduino file
coordinates_arduino_peaks = []
list_idx_arduino = 0  # Extra teller voor je lijsten
for p in range(len(arduino_peaks_selected)):
    if not arduino_peaks_selected[p]:
        coordinates_arduino_peaks.append([]) # Add new empty list
        fig,ax = plt.subplots()
        ax.plot(arduino_time[p], a_z[p], label="az", color="red")
        ax.set_title("Klik op de IMU pieken van de schouderophaal om de coordinaten op te slaan")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("az (m/s2)")
        line, = ax.plot(arduino_time[p], a_z[p], picker=5)

        def onclick(event):
            if event.artist == line:
                mouse_event = event.mouseevent
                xdata = mouse_event.xdata
                ydata = mouse_event.ydata

                idx = np.argmin(np.abs(arduino_time[p] - xdata))
                # -1 meaning, add to the last list
                coordinates_arduino_peaks[-1].append((arduino_time[p][idx], a_z[p][idx]))
                ax.plot(arduino_time[p][idx], a_z[p][idx], 'ro')
                fig.canvas.draw()

        cid = fig.canvas.mpl_connect("pick_event", onclick)

        plt.show()

        print("geselecteerde pieken:")
        for time, az_value in coordinates_arduino_peaks[list_idx_arduino]:
            print(f"  Piek: time {time} , az {az_value}")

        # Opslaan
        df = pd.DataFrame(coordinates_arduino_peaks[list_idx_arduino], columns=['time', 'az'])
        df.to_csv(f'{name}_pieken_IMU_{algorithm}_{p}.csv', index=False)

        print(f"Opgeslagen als '{name}_pieken_IMU_{algorithm}_{p}.csv'")

        list_idx_arduino += 1  # Vergeet niet de teller omhoog te doen!

### in coordinates_arduino_peaks staan de pieken van de az met tijd en hoogte van de piek, deze moeten nu gesyncroniseerd worden met de qualisys data.

# Inladen van piekbestanden arduino
arduino_peak_file = [None] * len(arduino_file)
for i in range(len(arduino_file)):
    # Check if the file exists before trying to read it
    try:
        arduino_peak_file[i] = pd.read_csv(f"{name}_pieken_IMU_{algorithm}_{i}.csv")
    except FileNotFoundError:
        print(f"File {name}_pieken_IMU_{algorithm}_{i}.csv not found. Skipping this file.")
        continue


## inladen van piekbestanden qualisys uitgaande van meerdere files.
# qualisys_peak_file = [None] * len(qualisys_file)
# for i in range(len(qualisys_file)):
#     # Check if the file exists before trying to read it
#     try:
#         qualisys_peak_file[i] = pd.read_csv(f"{name}_pieken_qualisys_{i}.csv")
#     except FileNotFoundError:
#         print(f"File {name}_pieken_qualisys_{i}.csv not found. Skipping this file.")
#         continue

## inladen van piekbestanden qualisys uitgaande van 1 file.
qualisys_peak_file = pd.read_csv(f"{name}_pieken_qualisys_{algorithm}_0.csv")



### count number of peaks in arduino data
arduino_number_of_peaks = [None] * len(arduino_file)
for i in range(len(arduino_file)):
    arduino_number_of_peaks[i] = len(arduino_peak_file[i])
    print(f"Number of peaks in {arduino_file[i]}: {arduino_number_of_peaks[i]}")

## asuming 1 qualisys file. if not, double for-loop needed
### qualisys data in evenveel stukken splitsen als arduino files
part_qualisys_peak_coordinates = [None] * len(arduino_file)
for i in range(len(arduino_file)):
    if i == 0:
        part_qualisys_peak_coordinates[i] = qualisys_peak_file[0:arduino_number_of_peaks[i]]
    if i == 1:
        part_qualisys_peak_coordinates[i] = qualisys_peak_file[arduino_number_of_peaks[i-1]:arduino_number_of_peaks[i-1]+arduino_number_of_peaks[i]]
    if i == 2:
        part_qualisys_peak_coordinates[i] = qualisys_peak_file[arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]:arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]+arduino_number_of_peaks[i]]
    if i ==3:
        part_qualisys_peak_coordinates[i] = qualisys_peak_file[arduino_number_of_peaks[i-3]+arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]:arduino_number_of_peaks[i-3]+arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]+arduino_number_of_peaks[i]]
    if i == 4:
        part_qualisys_peak_coordinates[i] = qualisys_peak_file[arduino_number_of_peaks[i-4]+arduino_number_of_peaks[i-3]+arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]:arduino_number_of_peaks[i-4]+arduino_number_of_peaks[i-3]+arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]+arduino_number_of_peaks[i]]
    if i == 5:
        part_qualisys_peak_coordinates[i] = qualisys_peak_file[arduino_number_of_peaks[i-5]+arduino_number_of_peaks[i-4]+arduino_number_of_peaks[i-3]+arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]:arduino_number_of_peaks[i-5]+arduino_number_of_peaks[i-4]+arduino_number_of_peaks[i-3]+arduino_number_of_peaks[i-2]+arduino_number_of_peaks[i-1]+arduino_number_of_peaks[i]]


    print(f"Peaks in part {i} of qualisys peaks: {len(part_qualisys_peak_coordinates[i])}")
    print(f"Peaks in part {i} of qualisys peaks: {part_qualisys_peak_coordinates[i]}")
    plt.plot(part_qualisys_peak_coordinates[i]['time'], part_qualisys_peak_coordinates[i]['motor_top'], 'ro', label=f'Part {i}')
    plt.plot(qualisys_time[0], prothese[0]["Motor_top"][:,1], label="prosthesis upper arm", color="green")
    plt.xlabel('Time (s)')
    plt.ylabel('Motor Top Position y-axis (mm)')
    plt.title(f'Qualisys Peaks Part {i}')
    plt.legend()
    plt.show()

# Bepaal tijdsverschil tussen eerste pieken (tijd compensatie) van arduino en qualisys
time_shift_peaks = [None] * len(arduino_file)
for i in range(len(arduino_file)):
    time_shift_peaks[i] = arduino_peak_file[i]['time'].iloc[0] - part_qualisys_peak_coordinates[i]['time'].iloc[0]
    print(f"Tijdverschil bestand {i}: {time_shift_peaks[i]:.4f} s")


## laad opnieuw alle arduino data in zodat deze geplot kan worden tegenover de qualisys data
for i in range(len(arduino_file)):
    df_full = pd.DataFrame({
        'bluetooth_time': bluetooth_time[i],
        'log_counter': log_counter[i],
        'PID_timestamp': PID_timestamp[i],
        'delta_PID_timestamp': delta_PID_timestamp[i],
        'elbow_angle': elbow_angle[i],
        'reference_velocity': reference_velocity[i],
        'raw_velocity': raw_velocity[i],
        'average_velocity': average_velocity[i],
        'PID_integral': PID_integral[i],
        'PID_derivative': PID_derivative[i],
        'output_velocity': output_velocity[i],
        'motor_speed_MPC': motor_speed_MPC[i],
        'ax': a_x[i],
        'ay': a_y[i],
        'az': a_z[i],
        'gx': g_x[i],
        'gy': g_y[i],
        'gz': g_z[i],
        'omega_x': omega_x[i],
        'arduino_time': arduino_time[i],
    })

    # Voeg berekeningen toe
    df_full['error_velocity'] = df_full['reference_velocity'] - df_full['average_velocity']
    df_full['motor_speed'] = df_full['output_velocity'] + df_full['motor_speed_MPC']

    # Tijd corrigeren
    df_full['arduino_time_shifted'] = df_full['arduino_time'] - time_shift_peaks[i]

    # Selecteer pieken
    peak_data = df_full[df_full['arduino_time'].isin(arduino_peak_file[i]['time'])].copy()
    peak_data['arduino_time_shifted'] = peak_data['arduino_time'] - time_shift_peaks[i]

    # Plotvoorbeeld gx
    fig, ax = plt.subplots(nrows=2, sharex=True, figsize=(10, 6))
    ax[0].plot(df_full['arduino_time_shifted'], df_full['az'], label="az full", color="lightblue")
    ax[0].plot(peak_data['arduino_time_shifted'], peak_data['az'], 'ro', label="az peaks")
    ax[0].set_title(f"Arduino az + peaks (File {i})")
    ax[0].set_xlabel("Time (s)")
    ax[0].set_ylabel("az (m/s2)")
    ax[0].legend()
    ax[0].grid(True)

    ax[1].plot(qualisys_time[0], prothese[0]["Motor_top"][:, 1], label="Prosthesis Upper Arm", color="green")
    ax[1].plot(part_qualisys_peak_coordinates[i]['time'], part_qualisys_peak_coordinates[i]['motor_top'], 'ro', label="qualisys peaks")
    ax[1].set_title(f"Qualisys Motor_top (File {i})")
    ax[1].set_xlabel("Time (s)")
    ax[1].set_ylabel("Motor Top Position y-axis (mm)")
    ax[1].legend()
    ax[1].grid(True)

    plt.tight_layout()
    plt.show()
