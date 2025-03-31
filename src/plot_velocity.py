### DO NOT TOUCH ###


import matplotlib.pyplot as plt
from matplotlib import colormaps
from collections import defaultdict
import numpy as np
import scipy.signal as signal
import math
import re
import os

# The current column names are as follows:
# ['python_time', 'N', 't', 'dt', 'REF', 'ENC_RAW', 'ENC_DEG', 'VEL_RAW', 'VEL_AVG', 'VEL_ERR', 'I', 'D', 'MS']
# I put this here simply so that copilot can help us with the column names

def find_latest_logfile(offset=0):
    log_folder = "./logs"
    all_logfiles = os.listdir(log_folder)
    all_logfiles = [ f for f in all_logfiles if f.startswith("log_2025")]
    log_filename = sorted(all_logfiles, reverse=True)[offset]
    return os.path.join(log_folder, log_filename)

def load_log_file_without_column_names(filepath):
    all_numbers = []
    lines = open(filepath, "r").readlines()
    # Rmove any empty lines
    lines = [line for line in lines if line.strip() != ""]

    for line in lines:
        # Find everything in the line that is made up of a bunch of numbers and a .
        matches = re.findall("-?[0-9\.]+", line)
        numbers = [ float(m) for m in matches ]
        all_numbers.append(numbers)
    return np.array(all_numbers)

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

# logfile = find_latest_logfile(0)
logfile = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250304_220730.txt"
# print(f"Now opening {logfile}")
# column_names, values = load_log_file(logfile)

values = load_log_file_without_column_names(logfile)
column_names = ["python_time", "t", "ENC_RAW", "MS", "VEL_RAW"]


encoder_velocities = get(values, "VEL_RAW")
motor_speeds = get(values, "MS")
timestamps = get(values, "t")

# Turn -12, -11, -14, 16, 12, 9 into -1, -1, -1, 1, 1, 1
encoder_signs = np.sign(encoder_velocities - 0.0000001) # The -0.0000001 is needed because sometimes velocities are exactly 0 and that messes with detection from + to - or from - to +
# Turn -1, -1, -1, 1, 1, 1 into 0, 0, 0, 2, 0, 0
encoder_changes = np.diff(encoder_signs)
# Find all the 2's and -2's. That's where we flip from + to - or from - to +
encoder_flips_up = [0] + list(np.where(encoder_changes == 2)[0])
encoder_flips_down = list(np.where(encoder_changes == -2)[0])
# Find all the timestamps where we flip
encoder_flips_up_t = [ timestamps[idx] for idx in encoder_flips_up ]
encoder_flips_down_t = [ timestamps[idx] for idx in encoder_flips_down ]

# For each region between an up-flip and down-flip
for i, (up, down) in enumerate(zip(encoder_flips_up, encoder_flips_down)):
    timestamps_partial = timestamps[up+1:down]
    encoder_velocities_partial = encoder_velocities[up+1:down]
    encoder_velocities_average = create_moving_average(encoder_velocities_partial, 15)
    motor_speed = motor_speeds[up+1]

    # Fancy colors
    cmap = plt.cm.tab20
    color = cmap(i%20)

    plt.plot(timestamps_partial, encoder_velocities_partial, color=color, alpha=0.5)
    plt.plot(timestamps_partial, encoder_velocities_average, color=color, label=f"{int(motor_speed)}")

plt.ylim(0, 30)
plt.grid()
plt.legend(bbox_to_anchor=(1.00, 1), loc="upper left",ncol=3)#(ncol=10)
# plt.tight_layout()
plt.show()


exit()

# velocity_error = setpoint - raw_velocity
# velocity_error_cumulative = np.cumsum(velocity_error)

# # Create moving average with window size of 15
# window_size = 15
# moving_average = np.convolve(raw_velocity, np.ones(window_size)/window_size, mode='valid')
# # Add zeros to the beginning and end of the moving average to make it the same length as the raw velocity
# moving_average = np.concatenate((np.zeros(window_size//2), moving_average, np.zeros(window_size//2))) 


# ### Plot moving average and motor speed in the same plot but on different y-axis

# # Time is in microseconds, convert to seconds and make it start from 0
# time = get(values, "t") / 1000000.0
# time -= time[0]

# fig, ax1 = plt.subplots()
# ax1.set_xlabel('Time (s)')
# ax1.set_ylabel('Motor Speed (RPM)', color='tab:blue')
# ax1.set_ylim([21000, 31400])
# ax1.yaxis.set_ticks(list(np.arange(21000, 31400, 1000)) + [31400])
# ax1.plot(time, get(values, "MS"), color='tab:blue', label="Motor Speed (RPM)")
# ax1.tick_params(axis='y', labelcolor='tab:blue')
# ax1.legend(loc='upper left')

# ax2 = ax1.twinx()
# ax2.set_ylabel('Speed (deg/s)', color='tab:red')
# ax2.set_ylim([-25, 25])
# ax2.yaxis.set_ticks(list(np.arange(-25, 25, 5)) + [25])
# ax2.plot(time, moving_average, color='tab:red', label="Speed Moving Average (15) (deg/s)")
# ax2.plot(time, get(values, "REF"), color='tab:orange', label="Speed Reference (deg/s)")
# ax2.tick_params(axis='y', labelcolor='tab:red')
# ax2.legend(loc='upper right')

# ax1.title.set_text("Motor Speed and Speed Moving Average")
# fig.tight_layout()
# plt.grid()
# plt.legend()
# plt.show()




# import matplotlib.pyplot as plt
# import numpy as np

# timestamps = []
# encoder_velocities = []
# elbow_angles = []
# motorspeeds = []

# bestandsnaam = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250304_220730.txt"
# with open(bestandsnaam, "r") as file:
#     lines = file.readlines()
#     for line in lines:
#         line = line.strip()
#         if " " in line:  # Make sure there's a space to split
#             bluetooth_time, data = line.split(" ", 1)  
#             timestamp, elbow_angle, motorspeed, encoder_velocity = map(float, data.split(",")) 

#             timestamps.append(timestamp)
#             encoder_velocities.append(encoder_velocity)
#             motorspeeds.append(motorspeed)

# encoder_velocities = np.array(encoder_velocities)
# motorspeeds = np.array(motorspeeds)

# unique_motorspeeds = np.unique(motorspeeds)
# for speed in unique_motorspeeds:
#     indices = np.where(motorspeeds == speed)
#     timestamps = np.array(timestamps)  # Convert timestamps to a NumPy array
#     plt.plot(timestamps[indices], encoder_velocities[indices], label=f"{speed}")
# plt.legend()
# plt.grid()
# plt.show()
# exit()



# encoder_signs = np.sign(encoder_velocities)
# encoder_changes = np.diff(encoder_signs)
# encoder_flips_up = [0] + list(np.where(encoder_changes > 0)[0])
# encoder_flips_down = list(np.where(encoder_changes < 0)[0])

# print("\nencoder_flips_up")
# print(encoder_flips_up)

# print("\nencoder_flips_down")
# print(encoder_flips_down)

# for i, (up, down) in enumerate(zip(encoder_flips_up, encoder_flips_down)):
#     print(f"Now plotting from value {up} to {down}")
#     plt.plot(timestamps[up+1:down], encoder_velocities[up+1:down] + 10 * i)
# plt.show()

# exit()




# # Bepaal de grootte van het venster voor de moving average
# window_size = 10

# # Bereken de moving average met numpy
# encoder_velocities_smooth = np.convolve(encoder_velocities, np.ones(window_size)/window_size, mode='valid')

# # Aanpassen van timestamps om lengte gelijk te maken
# timestamps_smooth = timestamps[:len(encoder_velocities_smooth)]


# plt.plot(timestamps_smooth, encoder_velocities_smooth)
# plt.grid()
# plt.show()
