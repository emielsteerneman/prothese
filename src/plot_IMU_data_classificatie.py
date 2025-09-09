## CODE IS GESCHIKT VOOR CLASSIFICATIE VAN PROTHESE GEDRAG, TP/FP/TN/FN

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
algorithm = "Alg1"

qualisys_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_{algorithm}.csv"
qualisys_df = pd.read_csv(qualisys_file)

qualisys_peaks_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_qualisys_{algorithm}_0.csv"
qualisys_peaks_df = pd.read_csv(qualisys_peaks_file)

imu_trial_files = [
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_0.csv",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_1.csv",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_2.csv",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_3.csv",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_4.csv",
    # f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_pieken_IMU_{algorithm}_5.csv"
]

imu_raw_files = [
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_1.txt",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_2.txt",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_3.txt",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_4.txt",
    f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\IMU_data\\{name}\\{algorithm}\\{algorithm}_5.txt",
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

import numpy as np

class MahonyAHRS:
    def __init__(self, sample_freq=50.0, two_kp=2.0 * 25.0, two_ki=2.0 * 0.0001):
        self.two_kp = two_kp
        self.two_ki = two_ki
        self.inv_sample_freq = 1.0 / sample_freq
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.integral_fb = np.array([0.0, 0.0, 0.0])
        self.angles_computed = False
        self.roll = 0.0  # radians

    def inv_sqrt(self, x):
        return 1.0 / np.sqrt(x)

    def update_imu(self, gx, gy, gz, ax, ay, az):
        q0, q1, q2, q3 = self.q

        # Convert gyro deg/s to rad/s
        gx = np.radians(gx)
        gy = np.radians(gy)
        gz = np.radians(gz)

        # Accelerometer validity check
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            norm = self.inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= norm
            ay *= norm
            az *= norm

            # Estimated direction of gravity (half vectors)
            half_vx = q1 * q3 - q0 * q2
            half_vy = q0 * q1 + q2 * q3
            half_vz = q0 * q0 - 0.5 + q3 * q3

            # Error is sum of cross product
            half_ex = ay * half_vz - az * half_vy
            half_ey = az * half_vx - ax * half_vz
            half_ez = ax * half_vy - ay * half_vx

            # Apply integral feedback
            if self.two_ki > 0.0:
                self.integral_fb += np.array([half_ex, half_ey, half_ez]) * self.two_ki * self.inv_sample_freq
                gx += self.integral_fb[0]
                gy += self.integral_fb[1]
                gz += self.integral_fb[2]
            else:
                self.integral_fb[:] = 0.0

            # Apply proportional feedback
            gx += self.two_kp * half_ex
            gy += self.two_kp * half_ey
            gz += self.two_kp * half_ez

        # Integrate rate of change of quaternion
        gx *= 0.5 * self.inv_sample_freq
        gy *= 0.5 * self.inv_sample_freq
        gz *= 0.5 * self.inv_sample_freq

        qa, qb, qc = q0, q1, q2
        q0 += (-qb * gx - qc * gy - q3 * gz)
        q1 += (qa * gx + qc * gz - q3 * gy)
        q2 += (qa * gy - qb * gz + q3 * gx)
        q3 += (qa * gz + qb * gy - qc * gx)

        # Normalize quaternion
        norm = self.inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = np.array([q0, q1, q2, q3]) * norm
        self.angles_computed = False

    def compute_angles(self):
        q0, q1, q2, q3 = self.q
        self.roll = np.arctan2(q0 * q1 + q2 * q3, 0.5 - q1 * q1 - q2 * q2)
        self.angles_computed = True

    def get_roll(self, degrees=True):
        if not self.angles_computed:
            self.compute_angles()
        return np.degrees(self.roll) if degrees else self.roll
    
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

colors = ['blue', 'green', 'coral', 'purple', 'pink', 'gray']
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

#     ax.plot(df_full['arduino_time_shifted'], df_full['az'], label=f"az full (File {i})", color=colors[i])
#     ax.plot(peak_data['arduino_time_shifted'], peak_data['az'], 'o', label=f"az peaks (File {i})", color=peak_colors[i])
#     ax.axvline(x=imu_start_times[i] - time_shift_peaks[i], color=peak_colors[i], linestyle='--', label=f"Starttijd (File {i})")

# ax.set_title("az over tijd + starttijd lijnen (alle bestanden)")
# ax.set_xlabel("Tijd (s)")
# ax.set_ylabel("az (m/s²)")
# ax.grid(True)
# ax.legend()
# plt.tight_layout()
# plt.show()

qualisys_peak_index = 0
global_trial_index = 0  # toegevoegde globale teller

import pandas as pd

def filter_cooldown_events(df_trial, cooldown=0.5):
    triggers = df_trial[df_trial["crossing"] != 0].copy()
    triggers = triggers.sort_values("time").reset_index(drop=True)

    filtered_rows = []

    for direction in [1, -1]:  # apart behandelen
        dir_triggers = triggers[triggers["crossing"] == direction].copy()
        dir_filtered = []
        last_trigger_time = -float('inf')

        for i, row in dir_triggers.iterrows():
            if row["time"] - last_trigger_time >= cooldown:
                dir_filtered.append(row)
                last_trigger_time = row["time"]

        filtered_rows.extend(dir_filtered)

    try:
        return pd.DataFrame(filtered_rows).sort_values("time").reset_index(drop=True)
    except Exception:
        print("No valid events found after filtering.")
        return None

# Tellers

# total_up_events = 0
# total_down_events = 0
# total_up_matches = 0
# total_down_matches = 0

# for file_i in range(len(imu_raw_files)):
#     n_peaks = arduino_number_of_peaks[file_i]
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_gx = g_x[file_i]
#     imu_ref_vel = reference_velocity[file_i]
#     duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values

#     for trial_i in range(n_peaks):
#         start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
#         duur = duur_trials[trial_i]

#         mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
#         times_trial = imu_time_shifted[mask]
#         gx_trial = imu_gx[mask]
#         ref_vel_trial = imu_ref_vel[mask]

#         df_trial = pd.DataFrame({
#             "time": times_trial,
#             "gyro_x": gx_trial,
#             "reference_velocity": ref_vel_trial
#         })

#         # crossings detecteren
#         gx_prev = df_trial["gyro_x"].shift(1)
#         crossing_up = ((gx_prev < 80) & (df_trial["gyro_x"] >= 80))
#         crossing_down = ((gx_prev > -80) & (df_trial["gyro_x"] <= -80))
#         df_trial["crossing"] = 0
#         df_trial.loc[crossing_up, "crossing"] = 1
#         df_trial.loc[crossing_down, "crossing"] = -1

#         df_trial["ref_change"] = df_trial["reference_velocity"].ne(df_trial["reference_velocity"].shift())
#         df_trial["last_ref"] = df_trial.loc[df_trial["ref_change"], "reference_velocity"]
#         df_trial["last_ref"] = df_trial["last_ref"].ffill()

#         print(f"Global trial {global_trial_index} (local trial {trial_i} in file {file_i}): totale crossings vóór filter: {df_trial['crossing'].sum()} (up={sum(df_trial['crossing']==1)}, down={sum(df_trial['crossing']==-1)})")

#         filtered_events = filter_cooldown_events(df_trial)
#         print(f"Global trial {global_trial_index}: crossings na filter: {len(filtered_events)} (up={len(filtered_events[filtered_events['crossing']==1])}, down={len(filtered_events[filtered_events['crossing']==-1])})")

#         if len(filtered_events) > 0 and trial_i == 0:
#             import matplotlib.pyplot as plt
#             plt.figure(figsize=(10, 4))
#             plt.plot(df_trial["time"], df_trial["gyro_x"], label="gyro_x")
#             plt.plot(df_trial["time"], df_trial["reference_velocity"], label="reference_velocity")
#             plt.scatter(filtered_events["time"], [0]*len(filtered_events), color='red', label="crossings")
#             plt.legend()
#             plt.title(f"Crossings en reference_velocity — trial {global_trial_index}")
#             plt.show()

#         for _, row in filtered_events.iterrows():
#             t = row["time"]
#             direction = row["crossing"]

#             last_known_ref = df_trial.loc[df_trial["time"] <= t, "last_ref"].iloc[-1]

#             expected = None
#             if direction == 1:  # up
#                 if last_known_ref == 0:
#                     expected = 12.5
#                 elif last_known_ref == 12.5:
#                     expected = 0
#                 if expected is not None:
#                     total_up_events += 1
#             elif direction == -1:  # down
#                 if last_known_ref == 0:
#                     expected = -12.5
#                 elif last_known_ref == -12.5:
#                     expected = 0
#                 if expected is not None:
#                     total_down_events += 1

#             if expected is not None:
#                 window_mask = (df_trial["time"] >= t - 0.3) & (df_trial["time"] <= t + 0.3)
#                 ref_window = df_trial.loc[window_mask, "reference_velocity"]
#                 match_found = any(abs(val - expected) < 0.01 for val in ref_window if not pd.isna(val))

#                 print(f"  Event at t={t:.3f}, direction={direction}, last_ref={last_known_ref}, expected={expected}, match_found={match_found}")
#                 if match_found:
#                     if direction == 1:
#                         total_up_matches += 1
#                     elif direction == -1:
#                         total_down_matches += 1

#         qualisys_peak_index += 1
#         global_trial_index += 1  # verhoog globale teller per trial

# # Eindresultaten
# if total_up_events > 0:
#     print(f"UP correct: {total_up_matches}/{total_up_events} ({100 * total_up_matches / total_up_events:.1f}%)")
# else:
#     print("UP correct: 0/0 (geen events gevonden)")

# if total_down_events > 0:
#     print(f"DOWN correct: {total_down_matches}/{total_down_events} ({100 * total_down_matches / total_down_events:.1f}%)")
# else:
#     print("DOWN correct: 0/0 (geen events gevonden)")

import matplotlib.pyplot as plt

qualisys_peak_index = 0

# colors = ['blue', 'green', 'red']

# Lijsten om correcte en foutieve events absolute tijd te bewaren
correct_up_times = []
wrong_up_times = []
correct_down_times = []
wrong_down_times = []

for file_i in range(len(imu_raw_files)):
    n_peaks = arduino_number_of_peaks[file_i]
    imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
    imu_gx = g_x[file_i]
    imu_ref_vel = reference_velocity[file_i]
    duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values

    for trial_i in range(n_peaks):
        start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
        duur = duur_trials[trial_i]

        mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        times_trial = imu_time_shifted[mask]
        gx_trial = imu_gx[mask]
        ref_vel_trial = imu_ref_vel[mask]

        # Bouw DataFrame en detecteer crossings (zoals in jouw code)
        df_trial = pd.DataFrame({
            "time": times_trial,
            "gyro_x": gx_trial,
            "reference_velocity": ref_vel_trial
        })

        gx_prev = df_trial["gyro_x"].shift(1)
        crossing_up = ((gx_prev < 75) & (df_trial["gyro_x"] >= 75))
        crossing_down = ((gx_prev > -75) & (df_trial["gyro_x"] <= -75   ))
        df_trial["crossing"] = 0
        df_trial.loc[crossing_up, "crossing"] = 1
        df_trial.loc[crossing_down, "crossing"] = -1

        df_trial["ref_change"] = df_trial["reference_velocity"].ne(df_trial["reference_velocity"].shift())
        df_trial["last_ref"] = df_trial.loc[df_trial["ref_change"], "reference_velocity"]
        df_trial["last_ref"] = df_trial["last_ref"].ffill()

        filtered_events = filter_cooldown_events(df_trial)

        if  filtered_events is not None:
            for _, row in filtered_events.iterrows():
                t = row["time"]
                direction = row["crossing"]
                last_known_ref = df_trial.loc[df_trial["time"] <= t, "last_ref"].iloc[-1]

                expected = None
                if direction == 1:  # up
                    if last_known_ref == 0:
                        expected = 12.5
                    elif last_known_ref == 12.5:
                        expected = 0
                elif direction == -1:  # down
                    if last_known_ref == 0:
                        expected = -12.5
                    elif last_known_ref == -12.5:
                        expected = 0

                if expected is not None:
                    window_mask = (df_trial["time"] >= t - 0.4) & (df_trial["time"] <= t + 0.4)
                    ref_window = df_trial.loc[window_mask, "reference_velocity"]
                    match_found = any(abs(val - expected) < 0.01 for val in ref_window if not pd.isna(val))

                    if direction == 1:
                        if match_found:
                            correct_up_times.append(t)
                        else:
                            wrong_up_times.append(t)
                    elif direction == -1:
                        if match_found:
                            correct_down_times.append(t)
                        else:
                            wrong_down_times.append(t)

        qualisys_peak_index += 1


# Nu plotten volgens jouw stijl met bolletjes:

qualisys_peak_index = 0

fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

sample_freq = 50.0  # jouw samplefrequentie, pas aan indien nodig


for file_i in range(len(imu_raw_files)):
    n_peaks = arduino_number_of_peaks[file_i]
    imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
    imu_gx = g_x[file_i]
    imu_ax = a_x[file_i]
    imu_ay = a_y[file_i]
    imu_az = a_z[file_i]
    imu_gx = g_x[file_i]
    imu_gy = g_y[file_i]
    imu_gz = g_z[file_i]
    imu_ref_vel = reference_velocity[file_i]
    duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    imu_elbow_angle = elbow_angle[file_i]

    for trial_i in range(n_peaks):
        start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
        duur = duur_trials[trial_i]

        mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)

        times_trial = imu_time_shifted[mask]
        gx_trial = imu_gx[mask]
        ref_vel_trial = imu_ref_vel[mask]
        elbow_angle_trial = imu_elbow_angle[mask]
        times_trial = imu_time_shifted[mask]

        ax_trial = imu_ax[mask]
        ay_trial = imu_ay[mask]
        az_trial = imu_az[mask]
        gx_trial = imu_gx[mask]
        gy_trial = imu_gy[mask]
        gz_trial = imu_gz[mask]
        ref_vel_trial = imu_ref_vel[mask]
        
        # AHRS init
        mahony = MahonyAHRS(sample_freq=sample_freq)


        roll_angles = []
        for gx, gy, gz, ax, ay, az in zip(gx_trial, gy_trial, gz_trial, ax_trial, ay_trial, az_trial):
            mahony.update_imu(gx, gy, gz, ax, ay, az)
            roll = mahony.get_roll(degrees=True)
            roll_angles.append(roll)

        axs[0].plot(times_trial, gx_trial, color=colors[file_i % len(colors)], alpha=0.7)
        axs[0].plot(times_trial, [100] * len(times_trial), color='orange', linestyle='--')
        axs[0].plot(times_trial, [-100] * len(times_trial), color='orange', linestyle='--')
        axs[0].plot(times_trial, [80] * len(times_trial), color='red', linestyle='--')
        axs[0].plot(times_trial, [-80] * len(times_trial), color='red', linestyle='--')
        axs[0].plot(times_trial, elbow_angle_trial, color='purple')
        axs[0].plot(times_trial, [90] * len(times_trial), color='blue', linestyle='--')
        axs[1].plot(times_trial, ref_vel_trial, color=colors[file_i % len(colors)], alpha=0.7)
        axs[1].plot(times_trial, roll_angles, color='purple', label='Roll Angle', alpha=0.7)
        axs[1].plot(times_trial, [20] * len(times_trial), color='black', linestyle='--')

        qualisys_peak_index += 1

# Voeg bolletjes toe op correcte en foutieve detecties (absolute tijd)
axs[0].scatter(correct_up_times, [110] * len(correct_up_times), color='green', marker='o', label='Correct UP')
axs[0].scatter(wrong_up_times, [110] * len(wrong_up_times), color='red', marker='x', label='Wrong UP')
axs[0].scatter(correct_down_times, [-110] * len(correct_down_times), color='blue', marker='o', label='Correct DOWN')
axs[0].scatter(wrong_down_times, [-110] * len(wrong_down_times), color='orange', marker='x', label='Wrong DOWN')

axs[0].set_title("gx per trial, absolute tijd met event detecties")
axs[0].set_ylabel("gx (rad/s)")
axs[0].grid(True)
axs[0].legend(loc='upper right')

axs[1].set_title("Reference velocity per trial, absolute tijd")
axs[1].set_xlabel("Tijd (s)")
axs[1].set_ylabel("Reference velocity")
axs[1].grid(True)

plt.tight_layout()
plt.show()

# plot de correcte en foutieve ups en downs in een figuur
# qualisys_peak_index = 0

# import matplotlib.pyplot as plt

# colors = ['blue', 'green', 'red']


# fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# for file_i in range(len(imu_raw_files)):
#     n_peaks = arduino_number_of_peaks[file_i]
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_gx = g_x[file_i]
#     imu_ref_vel = reference_velocity[file_i]
#     duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
#     for trial_i in range(n_peaks):
#         start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
#         duur = duur_trials[trial_i]
        
#         mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        
#         times_trial = imu_time_shifted[mask]
#         gx_trial = imu_gx[mask]
#         ref_vel_trial = imu_ref_vel[mask]
        
#         axs[0].plot(times_trial, gx_trial, color=colors[file_i], alpha=0.7)
#         axs[0].plot(times_trial, [100] * len(times_trial), color='orange', linestyle='--')
#         axs[0].plot(times_trial, [-100] * len(times_trial), color='orange', linestyle='--')
#         axs[1].plot(times_trial, ref_vel_trial, color=colors[file_i], alpha=0.7)
        
#         qualisys_peak_index += 1

# axs[0].set_title("gx per trial, absolute tijd")
# axs[0].set_ylabel("gx (rad/s)")
# axs[0].grid(True)

# axs[1].set_title("Reference velocity per trial, absolute tijd")
# axs[1].set_xlabel("Tijd (s)")
# axs[1].set_ylabel("Reference velocity")
# axs[1].grid(True)

# plt.tight_layout()
# plt.show()







# plt.figure(figsize=(14, 7))

# colors = ['blue', 'green', 'red']

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


# plt.figure(figsize=(14, 7))
# colors = ['blue', 'green', 'red']

# qualisys_peak_index = 0
# for file_i in range(len(imu_raw_files)):
#     n_peaks = arduino_number_of_peaks[file_i]
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_gx = g_x[file_i]
#     duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
#     for trial_i in range(n_peaks):
#         start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
#         duur = duur_trials[trial_i]
        
#         mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
#         times_trial = imu_time_shifted[mask]
#         gx_trial = imu_gx[mask]
        
#         plt.plot(times_trial, gx_trial, color=colors[file_i], alpha=0.7)
        
#         qualisys_peak_index += 1

# plt.title("gx per trial, absolute tijd (niet overlappend)")
# plt.xlabel("Tijd (s)")
# plt.ylabel("gx (rad/s)")
# plt.grid(True)
# plt.show()

# import matplotlib.pyplot as plt

# colors = ['blue', 'green', 'red']


# fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# for file_i in range(len(imu_raw_files)):
#     n_peaks = arduino_number_of_peaks[file_i]
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_gx = g_x[file_i]
#     imu_ref_vel = reference_velocity[file_i]
#     duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
#     for trial_i in range(n_peaks):
#         start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
#         duur = duur_trials[trial_i]
        
#         mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        
#         times_trial = imu_time_shifted[mask]
#         gx_trial = imu_gx[mask]
#         ref_vel_trial = imu_ref_vel[mask]
        
#         axs[0].plot(times_trial, gx_trial, color=colors[file_i], alpha=0.7)
#         axs[0].plot(times_trial, [100] * len(times_trial), color='orange', linestyle='--')
#         axs[0].plot(times_trial, [-100] * len(times_trial), color='orange', linestyle='--')
#         axs[1].plot(times_trial, ref_vel_trial, color=colors[file_i], alpha=0.7)
        
#         qualisys_peak_index += 1

# axs[0].set_title("gx per trial, absolute tijd")
# axs[0].set_ylabel("gx (rad/s)")
# axs[0].grid(True)

# axs[1].set_title("Reference velocity per trial, absolute tijd")
# axs[1].set_xlabel("Tijd (s)")
# axs[1].set_ylabel("Reference velocity")
# axs[1].grid(True)

# plt.tight_layout()
# plt.show()

# import matplotlib.pyplot as plt

# colors = ['blue', 'green', 'red']
# qualisys_peak_index = 0

# fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# for file_i in range(len(imu_raw_files)):
#     n_peaks = arduino_number_of_peaks[file_i]
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_gx = g_x[file_i]
#     imu_elbow = elbow_angle[file_i]
#     duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
#     for trial_i in range(n_peaks):
#         start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
#         duur = duur_trials[trial_i]
        
#         mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        
#         times_trial = imu_time_shifted[mask]
#         gx_trial = imu_gx[mask]
#         elbow_trial = imu_elbow[mask]
        
#         axs[0].plot(times_trial, gx_trial, color=colors[file_i], alpha=0.7)
#         axs[1].plot(times_trial, elbow_trial, color=colors[file_i], alpha=0.7)
        
#         qualisys_peak_index += 1

# axs[0].set_title("gx per trial, absolute tijd")
# axs[0].set_ylabel("gx (rad/s)")
# axs[0].grid(True)

# axs[1].set_title("Elbow angle per trial, absolute tijd")
# axs[1].set_xlabel("Tijd (s)")
# axs[1].set_ylabel("Elbow angle (deg)")
# axs[1].grid(True)

# plt.tight_layout()
# plt.show()


    

# Roll berekenen met MahonyAHRS en plotten per trial
# fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# qualisys_peak_index = 0
# sample_freq = 50.0  # jouw samplefrequentie, pas aan indien nodig

# for file_i in range(len(imu_raw_files)):
#     n_peaks = arduino_number_of_peaks[file_i]
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_ax = a_x[file_i]
#     imu_ay = a_y[file_i]
#     imu_az = a_z[file_i]
#     imu_gx = g_x[file_i]
#     imu_gy = g_y[file_i]
#     imu_gz = g_z[file_i]
#     imu_ref_vel = reference_velocity[file_i]
#     duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
#     for trial_i in range(n_peaks):
#         start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
#         duur = duur_trials[trial_i]
        
#         mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        
#         times_trial = imu_time_shifted[mask]
#         ax_trial = imu_ax[mask]
#         ay_trial = imu_ay[mask]
#         az_trial = imu_az[mask]
#         gx_trial = imu_gx[mask]
#         gy_trial = imu_gy[mask]
#         gz_trial = imu_gz[mask]
#         ref_vel_trial = imu_ref_vel[mask]
        
#         # AHRS init
#         mahony = MahonyAHRS(sample_freq=sample_freq)
        
#         roll_angles = []
#         for gx, gy, gz, ax, ay, az in zip(gx_trial, gy_trial, gz_trial, ax_trial, ay_trial, az_trial):
#             mahony.update_imu(gx, gy, gz, ax, ay, az)
#             roll = mahony.get_roll(degrees=True)
#             roll_angles.append(roll)
        
#         print(file_i)
#         print(colors[file_i])
#         axs[0].plot(times_trial, roll_angles, color=colors[file_i], alpha=0.7)
#         axs[0].plot(times_trial, [20] * len(times_trial), color='orange', linestyle='--', label='20 graden referentie')
#         axs[1].plot(times_trial, ref_vel_trial, color=colors[file_i], alpha=0.7)
        
#         qualisys_peak_index += 1

# axs[0].set_title("Roll per trial, absolute tijd")
# axs[0].set_ylabel("Roll (graden)")
# axs[0].grid(True)

# axs[1].set_title("Reference velocity per trial, absolute tijd")
# axs[1].set_xlabel("Tijd (s)")
# axs[1].set_ylabel("Reference velocity")
# axs[1].grid(True)

# plt.tight_layout()
# plt.show()

# fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# sample_freq = 50.0  # pas aan indien nodig

# for file_i in range(len(imu_raw_files)):
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_ax = a_x[file_i]
#     imu_ay = a_y[file_i]
#     imu_az = a_z[file_i]
#     imu_gx = g_x[file_i]
#     imu_gy = g_y[file_i]
#     imu_gz = g_z[file_i]
#     imu_ref_vel = reference_velocity[file_i]

#     # AHRS init
#     mahony = MahonyAHRS(sample_freq=sample_freq)
    
#     roll_angles = []
#     for gx, gy, gz, ax, ay, az in zip(imu_gx, imu_gy, imu_gz, imu_ax, imu_ay, imu_az):
#         mahony.update_imu(gx, gy, gz, ax, ay, az)
#         roll = mahony.get_roll(degrees=True)
#         roll_angles.append(roll)
    
#     axs[0].plot(imu_time_shifted, roll_angles, color=colors[file_i], alpha=0.7, label=f'File {file_i}')
#     axs[0].plot(imu_time_shifted, imu_gx, color='purple', alpha=0.7, label=f'gx File {file_i}')
#     axs[0].plot(imu_time_shifted, [20] * len(imu_time_shifted), color='orange', linestyle='--', label='20 graden referentie')
#     axs[0].plot(imu_time_shifted, [-50] * len(imu_time_shifted), color='pink', linestyle='--')
#     axs[1].plot(imu_time_shifted, imu_ref_vel, color=colors[file_i], alpha=0.7, label=f'File {file_i}')

# axs[0].set_title("Roll over volledige tijd")
# axs[0].set_ylabel("Roll (graden)")
# axs[0].grid(True)

# axs[1].set_title("Reference velocity over volledige tijd")
# axs[1].set_xlabel("Tijd (s)")
# axs[1].set_ylabel("Reference velocity")
# axs[1].grid(True)

# axs[0].legend()
# axs[1].legend()

# plt.tight_layout()
# plt.show()

# plot raw velocity and reference velocity in one plot for all files in one plot oer trial
# colors = ['blue', 'green', 'red']
# qualisys_peak_index = 0

# plt.figure(figsize=(14, 7))
# # fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# for file_i in range(len(imu_raw_files)):
#     n_peaks = arduino_number_of_peaks[file_i]
#     imu_time_shifted = arduino_time[file_i] - time_shift_peaks[file_i]
#     imu_gx = g_x[file_i]
#     imu_elbow = elbow_angle[file_i]
#     imu_ref_vel = reference_velocity[file_i]
#     imu_raw_vel = raw_velocity[file_i]
#     imu_avg_vel = average_velocity[file_i]
#     duur_trials = part_qualisys_peak_coordinates[file_i]['Duur (s)'].values
    
#     for trial_i in range(n_peaks):
#         start_time = imu_start_times_per_trial[qualisys_peak_index] - time_shift_peaks[file_i]
#         duur = duur_trials[trial_i]
        
#         mask = (imu_time_shifted >= start_time) & (imu_time_shifted <= start_time + duur)
        
#         times_trial = imu_time_shifted[mask]
#         gx_trial = imu_gx[mask]
#         elbow_trial = imu_elbow[mask]
#         ref_vel_trial = imu_ref_vel[mask]
#         raw_vel_trial = imu_raw_vel[mask]
#         avg_vel_trial = imu_avg_vel[mask]
        
#         plt.plot(times_trial, raw_vel_trial, color=colors[file_i], alpha=0.7, label=f'Raw Velocity File {file_i}')
#         plt.plot(times_trial, ref_vel_trial, color="orange", linestyle='--', label=f'Reference Velocity File {file_i}')
#         plt.plot(times_trial, avg_vel_trial, color=colors[file_i], label=f'Average Velocity File {file_i}')

        
#         qualisys_peak_index += 1

# plt.title("Raw Velocity, Reference Velocity en Average Velocity per trial, absolute tijd")
# plt.xlabel("Tijd (s)")
# plt.ylabel("Velocity (degrees/s)")
# plt.grid(True)
# # plt.legend()
# plt.tight_layout()
# plt.show()



# plt.figure(figsize=(14, 7))

# for i in range(len(imu_raw_files)):
#     plt.plot(arduino_time[i] - time_shift_peaks[i], raw_velocity[i], label=f'Raw Velocity File {i}', alpha=0.7)
#     plt.plot(arduino_time[i] - time_shift_peaks[i], reference_velocity[i], label=f'Reference Velocity File {i}', alpha=0.7)
#     plt.plot(arduino_time[i] - time_shift_peaks[i], average_velocity[i], label=f'Average Velocity File {i}', linestyle='--', alpha=0.7)

#     plt.axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
# plt.title('Raw Velocity vs Reference Velocity')
# plt.xlabel('Tijd (s)')
# plt.ylabel('Velocity (m/s)')
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.show()


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



