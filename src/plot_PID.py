import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
import scipy.signal as signal
import math
import re
import os

# The current column names are as follows:
# column_names = ['python_time', 'N', 't', 'dt', 'REF', 'ENC_RAW', 'dENC_RAW', 'ENC_DEG', 'VEL_RAW', 'VEL_AVG', 'VEL_ERR', 'I', 'D', 'Kp', 'Ki', 'Kd', 'MS']
# I put this here simply so that copilot can help us with the column names


# 0: bluetooth_time
# 1: log_counter,
# 2: PID_timestamp,
# 3: delta_PID_timestamp,
# 4: reference_velocity,
# 5: encoder_value,
# 6: delta_encoder_value
# 7: elbow_angle,
# 8: raw_velocity,
# 9: average_velocity,
# 10: error_velocity,
# 11: PID_integral,
# 12: PID_derivative,
#12.5: output_velocity
# 13: motor_speed


def find_latest_logfile(offset=0):
    log_folder = "./logs"
    all_logfiles = os.listdir(log_folder)
    all_logfiles = [ f for f in all_logfiles if f.startswith("log_2025")]
    log_filename = sorted(all_logfiles, reverse=True)[offset]
    return os.path.join(log_folder, log_filename)

# def load_log_file_without_column_names(filepath):
#     all_numbers = []
#     lines = open(filepath, "r").readlines()
#     # Rmove any empty lines
#     lines = [line for line in lines if line.strip() != ""]

#     for line in lines:
#         # Find everything in the line that is made up of a bunch of numbers and a .
#         matches = re.findall("-?[0-9\.]+", line)
#         numbers = [ float(m) for m in matches ]
#         all_numbers.append(numbers)
#     return np.array(all_numbers)

def load_log_file(filepath):
    all_numbers = []
    lines = open(filepath, "r").readlines()
    # Rmove any empty lines
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

logfile = find_latest_logfile(0)
# logfile = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250312_162604.txt"
print(f"Now opening {logfile}")
column_names, data = load_log_file(logfile)


bluetooth_time = get(data, "python_time")
log_counter = get(data, "N")
PID_timestamp = get(data, "t")
delta_PID_timestamp = get(data, "dt")
reference_velocity = get(data, "REF")
encoder_value = get(data, "ENC_RAW")
delta_encoder_value = get(data, "dENC_RAW")
elbow_angle = get(data, "ENC_DEG")
raw_velocity = get(data, "VEL_RAW")
average_velocity = get(data, "VEL_AVG")
error_velocity = get(data, "VEL_ERR")
# PID_integral = get(data, "I")
# PID_derivative = get(data, "D")
K_p = get(data, "Kp")
K_i = get(data, "Ki")
K_d = get(data, "Kd")
output_velocity = get(data, "VEL_OUT")
motor_speed_MPC = get(data, "VEL_MPC")
motor_speed = get(data, "MS")



plt.figure(figsize=(10, 6))
plt.plot((PID_timestamp-PID_timestamp.min())/1e6, reference_velocity, label="Reference velocity")
plt.plot((PID_timestamp-PID_timestamp.min())/1e6, raw_velocity, label="Raw velocity")
plt.plot((PID_timestamp-PID_timestamp.min())/1e6, average_velocity, label="Average velocity")
plt.xlim((PID_timestamp.min()-PID_timestamp.min())/1e6, (PID_timestamp.max()-PID_timestamp.min())/1e6)
# plt.ylim(-25,25)
# plt.ylim(-5,25)
plt.ylim(-25,5)
plt.xlabel("Time (s)")
plt.ylabel("Angular velocity ($^\circ$/s)")
plt.title(f"Angular Velocity, Kp: {K_p[0]}, Ki: {K_i[0]}, Kd: {K_d[0]}")
plt.grid()
plt.legend(bbox_to_anchor=(1.0, 0.7), loc="upper left")
plt.show()