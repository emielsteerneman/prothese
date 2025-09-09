## inladen van qualisys trial data voor duratie van trials
## inladen van imu trial data voor bepalen beginpunt
## inladen van ruwe imu data
## beginpunt kenmerken in ruwe imu data
## eindpunt kenmerken in ruwe imu data
## plotten van ruwe imu data over tijd
## input velocity, angular velocity, elbow angle, mahony roll (deze eerst nog uitrekenen)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import re

name = "WouterVisser"
algorithm = "Alg2"

qualisys_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_{algorithm}.csv"
qualisys_df = pd.read_csv(qualisys_file)

qualisys_peaks_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_qualisys_{algorithm}_0.csv"
qualisys_peaks_df = pd.read_csv(qualisys_peaks_file)

imu_trial_files = [
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_0.csv",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_1.csv",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_2.csv",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_3.csv",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_4.csv",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_5.csv"
]

imu_raw_files = [
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_1.txt",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_2.txt",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_3.txt",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_4.txt",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_5.txt",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_6.txt"
]

def load_log_file(filepath):
    all_numbers = []
    lines = open(filepath, "r").readlines()
    lines = [line for line in lines if line.strip() != ""]
    column_names = ["python_time"] + re.findall("[a-zA-Z_]+", lines[0])
    if len(column_names) == 1:
        raise ValueError("No column names found in the first line of the log file.")
    for line in lines:
        matches = re.findall("-?[0-9\.]+", line)
        numbers = [float(m) for m in matches]
        all_numbers.append(numbers)
    return column_names, np.array(all_numbers)

def get(data, column_name):
    return data[:, column_names.index(column_name)]

    
data = [None] * len(imu_raw_files)

for i in range(len(imu_raw_files)):
    column_names, data[i] = load_log_file(imu_raw_files[i])
    print(f"Data from {imu_raw_files[i]} loaded with {len(data[i])} rows and {len(column_names)} columns.")

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

for d in range(len(imu_raw_files)):
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
    arduino_time.append((PID_timestamp[d] - PID_timestamp[d][0]) / 1e6)

imu_raw_dfs = [pd.read_csv(f) for f in imu_raw_files]
imu_trial_dfs = [pd.read_csv(f) for f in imu_trial_files]
arduino_number_of_peaks = [len(df) for df in imu_trial_dfs]



for i in range(len(imu_trial_files)):
    print(f"Number of peaks in {imu_trial_files[i]}: {arduino_number_of_peaks[i]}")

part_qualisys_peak_coordinates = [None] * len(imu_trial_files)
for i in range(len(imu_trial_files)):
    if i == 0:
        part_qualisys_peak_coordinates[i] = qualisys_df[0:arduino_number_of_peaks[i]]
    else:
        start_idx = sum(arduino_number_of_peaks[:i])
        part_qualisys_peak_coordinates[i] = qualisys_df[start_idx:start_idx + arduino_number_of_peaks[i]]

time_shift_peaks = [None] * len(imu_trial_files)
for i in range(len(imu_trial_files)):
    time_shift_peaks[i] = imu_trial_dfs[i]['time'].iloc[0] - part_qualisys_peak_coordinates[i]['Starttijd (s)'].iloc[0]
    print(f"Tijdverschil bestand {i}: {time_shift_peaks[i]:.4f} s")

imu_start_times = []

for i in range(len(imu_trial_files)):
    qualisys_starttijd = part_qualisys_peak_coordinates[i]['Starttijd (s)'].iloc[0]
    # Pak juiste piektijd uit qualisys peaks bestand (rekening houdend met offset)
    qualisys_piektijd = qualisys_peaks_df['time'].iloc[sum(arduino_number_of_peaks[:i])]
    imu_piektijd = imu_trial_dfs[i]['time'].iloc[0]

    delta_start_to_peak = qualisys_starttijd - qualisys_piektijd
    imu_starttijd = imu_piektijd + delta_start_to_peak
    imu_start_times.append(imu_starttijd)

    print(f"[Trial {i}] Starttijd IMU: {imu_starttijd:.4f} s (delta: {delta_start_to_peak:.4f} s)")


imu_start_times_per_trial = []

# Totale telling voor index in piek-qualisys file
qualisys_peak_index = 0

for file_i in range(len(imu_trial_files)):
    n_peaks = arduino_number_of_peaks[file_i]
    
    # Deel qualisys starttijd voor deze trials
    qualisys_starttijd_trials = part_qualisys_peak_coordinates[file_i]['Starttijd (s)'].values
    
    # Deel qualisys piektijden uit het piekenbestand (ervan uitgaande dat ze in volgorde zijn)
    qualisys_piektijden_trials = qualisys_peaks_df['time'].iloc[qualisys_peak_index:qualisys_peak_index + n_peaks].values
    
    # IMU piektijden per trial
    imu_piektijden_trials = imu_trial_dfs[file_i]['time'].values
    
    for trial_i in range(n_peaks):
        delta_start_to_peak = qualisys_starttijd_trials[trial_i] - qualisys_piektijden_trials[trial_i]
        imu_starttijd_trial = imu_piektijden_trials[trial_i] + delta_start_to_peak
        
        imu_start_times_per_trial.append(imu_starttijd_trial)
        
        print(f"[File {file_i} Trial {trial_i}] Starttijd IMU: {imu_starttijd_trial:.4f} s (delta: {delta_start_to_peak:.4f} s)")
    
    qualisys_peak_index += n_peaks

# Plotten van alle data in één figuur met starttijd en pieken
# fig, ax = plt.subplots(figsize=(12, 6))

colors = ['lightblue', 'lightgreen', 'lightcoral', 'lightyellow', 'lightpink', 'lightgray']
peak_colors = ['blue', 'green', 'red', 'orange', 'purple', 'cyan']

for i in range(len(imu_raw_files)):
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

    df_full['error_velocity'] = df_full['reference_velocity'] - df_full['average_velocity']
    df_full['motor_speed'] = df_full['output_velocity'] + df_full['motor_speed_MPC']
    df_full['arduino_time_shifted'] = df_full['arduino_time'] - time_shift_peaks[i]

    peak_data = df_full[df_full['arduino_time'].isin(imu_trial_dfs[i]['time'])].copy()
    peak_data['arduino_time_shifted'] = peak_data['arduino_time'] - time_shift_peaks[i]



# plt.figure(figsize=(14, 7))

# colors = ['blue', 'green', 'red', 'orange', 'purple', 'cyan']

# for i in range(len(imu_raw_files)):
#     plt.plot(arduino_time[i] - time_shift_peaks[i], a_z[i], label=f'az File {i}', color=colors[i])
    
#     start_times_for_file = imu_start_times_per_trial[
#         sum(arduino_number_of_peaks[:i]) : sum(arduino_number_of_peaks[:i+1])
#     ]
#     start_times_for_file_shifted = [t - time_shift_peaks[i] for t in start_times_for_file]
    
#     for st in start_times_for_file_shifted:
#         plt.axvline(x=st, color=colors[i], linestyle='--', alpha=0.7)

# plt.title('Arduino az met trial starttijden (alle bestanden)')
# plt.xlabel('Tijd (s)')
# plt.ylabel('az (m/s²)')
# plt.legend()
# plt.grid(True)
# plt.show()



# plot raw velocity and reference velocity in one plot for all files in one plot per trial
colors = ['blue', 'green', 'red', 'orange', 'purple', 'cyan']
qualisys_peak_index = 0

plt.figure(figsize=(14, 7))
# fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

for file_i in range(len(imu_raw_files)):
    n_peaks = arduino_number_of_peaks[file_i]
    imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
    imu_gx = g_x[file_i]
    imu_elbow = elbow_angle[file_i]
    imu_ref_vel = reference_velocity[file_i]
    imu_raw_vel = raw_velocity[file_i]
    imu_avg_vel = average_velocity[file_i]
    duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
    for trial_i in range(n_peaks):
        start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
        duur = duur_trials[trial_i]
        
        mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        
        times_trial = imu_time_shifted[mask]
        gx_trial = imu_gx[mask]
        elbow_trial = imu_elbow[mask]
        ref_vel_trial = imu_ref_vel[mask]
        raw_vel_trial = imu_raw_vel[mask]
        avg_vel_trial = imu_avg_vel[mask]
        
        # plt.plot(times_trial, raw_vel_trial, color=colors[file_i], alpha=0.7, label=f'Raw Velocity File {file_i}')
        # plt.plot(times_trial, ref_vel_trial, color="orange", linestyle='--', label=f'Reference Velocity File {file_i}')
        # plt.plot(times_trial, avg_vel_trial, color=colors[file_i], label=f'Average Velocity File {file_i}')

        
        qualisys_peak_index += 1

# plt.title("Raw Velocity, Reference Velocity en Average Velocity per trial, absolute tijd")
# plt.xlabel("Tijd (s)")
# plt.ylabel("Velocity (degrees/s)")
# plt.grid(True)
# # plt.legend()
# plt.tight_layout()
# plt.show()

mae_per_trial = []
qualisys_peak_index = 0  # teller voor pieken over alle files

for file_i in range(len(imu_raw_files)):
    n_peaks = arduino_number_of_peaks[file_i]
    imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
    imu_ref_vel = reference_velocity[file_i]
    imu_avg_vel = average_velocity[file_i]
    imu_raw_vel = raw_velocity[file_i]
    imu_mpc_vel = motor_speed_MPC[file_i]
    imu_motor_speed = motor_speed[file_i]
    duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
    for trial_i in range(n_peaks):
        start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
        duur = duur_trials[trial_i]
        
        mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        
        ref_vel_trial = imu_ref_vel[mask]
        avg_vel_trial = imu_avg_vel[mask]
        motor_speed_trial = imu_motor_speed[mask]
        motor_speed_mpc_trial = imu_mpc_vel[mask]
        
        # mae = np.mean(np.abs(ref_vel_trial - avg_vel_trial))
        mae = np.mean(np.abs(motor_speed_mpc_trial - motor_speed_trial))
        mae_per_trial.append(mae)
        
        print(f"MAE trial {qualisys_peak_index} (File {file_i}, Trial {trial_i}): {mae:.4f}")
        
        qualisys_peak_index += 1

mean_mae = np.mean(mae_per_trial)
std_mae = np.std(mae_per_trial)

print(f"\nGemiddelde MAE over alle trials: {mean_mae:.4f}")
print(f"Standaarddeviatie van MAE over alle trials: {std_mae:.4f}")




### MODEL VELOCITY PLOT
# plt.figure(figsize=(14, 7))

# # for i in range(len(imu_raw_files)):
# plt.plot(arduino_time[0] + time_shift_peaks[0], motor_speed_MPC[0], label=f'Model predicted velocity File {1}', alpha=0.7)
# plt.plot(arduino_time[0] + time_shift_peaks[0], motor_speed[0], label=f'Output + model predicted velocity File {1}', alpha=0.7)
#     # plt.plot(arduino_time[i] - time_shift_peaks[i], average_velocity[i], label=f'Average Velocity File {i}', linestyle='--', alpha=0.7)

# plt.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
# plt.title('Model Velocity vs Output Velocity')
# plt.xlabel('Tijd (s)')
# plt.ylabel('Velocity (steps/s)')
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.show()



