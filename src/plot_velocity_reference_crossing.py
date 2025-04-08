import matplotlib.pyplot as plt
from matplotlib import colormaps
from collections import defaultdict
import numpy as np
import scipy.signal as signal
import math
import re
import os

window_size = 30

def find_latest_logfile(offset=0):
    log_folder = "./logs"
    all_logfiles = os.listdir(log_folder)
    all_logfiles = [ f for f in all_logfiles if f.startswith("log_2025")]
    log_filename = sorted(all_logfiles, reverse=True)[offset]
    return os.path.join(log_folder, log_filename)

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

def create_moving_average(values, window_size):
    average_values = []
    for i in range(len(values)):
        right = i + 1
        right = min(right, len(values))
        left = i - window_size + 1
        left = max(left, 0)
        average_values.append( np.sum(values[left:right]) / (right - left) )

    return average_values

#####################################################################################

logfile = find_latest_logfile(0)
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
# average_velocity = get(data, "VEL_AVG")
error_velocity = get(data, "VEL_ERR")
PID_integral = get(data, "I")
PID_derivative = get(data, "D")
K_p = get(data, "Kp")
K_i = get(data, "Ki")
K_d = get(data, "Kd")
motor_speed = get(data, "MS")

# Turn -12, -11, -14, 16, 12, 9 into -1, -1, -1, 1, 1, 1
encoder_signs = np.sign(raw_velocity - 0.0000001) # The -0.0000001 is needed because sometimes velocities are exactly 0 and that messes with detection from + to - or from - to +
# Turn -1, -1, -1, 1, 1, 1 into 0, 0, 0, 2, 0, 0
encoder_changes = np.diff(encoder_signs)
# Find all the 2's and -2's. That's where we flip from + to - or from - to +
encoder_flips_up = [0] + list(np.where(encoder_changes == 2)[0])
encoder_flips_down = list(np.where(encoder_changes == -2)[0])
# Find all the timestamps where we flip
encoder_flips_up_t = [ PID_timestamp[idx] for idx in encoder_flips_up ]
encoder_flips_down_t = [ PID_timestamp[idx] for idx in encoder_flips_down ]

######################################


for i, (up, down) in enumerate(zip(encoder_flips_up, encoder_flips_down)):
    timestamps_partial = PID_timestamp[up+1:down]
    raw_velocities_partial = raw_velocity[up+1:down]
    average_velocities_partial = create_moving_average(raw_velocities_partial, window_size)
    motor_speeds = motor_speed[up+1]
    elbow_angles_partial = elbow_angle[up+1:down]
    reference_velocity_partial = reference_velocity[up+1:down] 

    average_velocites_zeroed = average_velocities_partial - reference_velocity_partial
    average_velocites_zeroed_signs = np.sign(average_velocites_zeroed)
    average_velocites_zeroed_changes = np.diff(average_velocites_zeroed_signs)
    average_velocites_zeroed_flips_up = list(np.where(average_velocites_zeroed_changes == 2)[0])
    average_velocites_zeroed_flips_down = list(np.where(average_velocites_zeroed_changes == -2)[0])

    average_velocites_zeroed_flips = average_velocites_zeroed_flips_up + average_velocites_zeroed_flips_down
    average_velocites_zeroed_flips_t = [ timestamps_partial[idx] for idx in average_velocites_zeroed_flips ]

    #interpolate the timestamps to the average velocities
    timestamps_partial = np.array(timestamps_partial)
    average_velocities_partial = np.array(average_velocities_partial)
    

    # Fancy colors
    cmap = plt.cm.tab20
    color = cmap(i%20)


    plt.plot(timestamps_partial, raw_velocities_partial, color=color, alpha=0.5)
    plt.plot(timestamps_partial, average_velocities_partial, color=color, label=f"{int(motor_speeds)}")
    plt.scatter(average_velocites_zeroed_flips_t, [12.5]*len(average_velocites_zeroed_flips_t), color=color, marker="x", s=100)
plt.axhline(12.5)
plt.show()

































####### Plotting
# fig, (ax_angle, ax_vel) = plt.subplots(nrows=2, sharex=True, figsize=(10, 6))


# # For each region between an up-flip and down-flip
# for i, (up, down) in enumerate(zip(encoder_flips_up, encoder_flips_down)):
#     timestamps_partial = PID_timestamp[up+1:down]
#     encoder_velocities_partial = raw_velocity[up+1:down]
#     encoder_velocities_average = create_moving_average(encoder_velocities_partial, window_size)
#     motor_speeds = motor_speed[up+1]
#     elbow_angles_partial = elbow_angle[up+1:down]


#     # Fancy colors
#     cmap = plt.cm.tab20
#     color = cmap(i%20)


#     ax_vel.plot(timestamps_partial, encoder_velocities_partial, color=color, alpha=0.5)
#     ax_vel.plot(timestamps_partial, encoder_velocities_average, color=color, label=f"{int(motor_speeds)}")
#     ax_vel.plot(PID_timestamp, [12.5]*len(PID_timestamp))

#     # Angle subplot
#     ax_angle.plot(timestamps_partial, elbow_angles_partial, color=color, alpha=0.7)

# plt.ylim(0, 30)
# plt.grid()
# plt.legend(bbox_to_anchor=(1.00, 1), loc="upper left",ncol=3)#(ncol=10)
# plt.show()

# exit()

