### === eerste versie is klaar, nieuwe versie staat in chat === ###

# inladen van qualisys null data
# marker data van sternum uitlezen
# aantal frames bepalen per trial
# rotatie bepalen voor elke trial
# translatie bepalen voor elke trial
# gemiddelde per target bepalen
# trials interpoleren (maar vorm behouden)
# trials normaliseren 
# algoritme data inladen
# data interpoleren
# data normaliseren t.o.v. null data


import json
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

name = "AnnaZoet"
algorithm = "Null"
N = 100  # gekozen resolutie voor interpolatie #84 voor Null Anna, 576 voor Alg1 Anna


# Pad naar JSON-bestand
json_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
csv_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_{algorithm}.csv"

def r_svd(base1, base2):
    base1_mean = np.mean(base1, axis=1, keepdims=True)
    base2_mean = np.mean(base2, axis=1, keepdims=True)
    base1_zero = base1 - base1_mean
    base2_zero = base2 - base2_mean
    u_arr, _, vh_arr = np.linalg.svd(base2_zero @ base1_zero.T)
    r_mat = (u_arr * np.array([1, 1, np.linalg.det(u_arr @ vh_arr)])) @ vh_arr
    trans = base2_mean - r_mat @ base1_mean
    error = ((base2 - (r_mat @ base1 + trans)) ** 2).mean() ** 0.5
    return r_mat, trans, error

# Marker namen van de sternum cluster
sternum_markers = ["Ster_cen", "Ster_R", "Ster_L"]

# Inlezen van JSON-data
with open(json_path, 'r') as f:
    data = json.load(f)


# Extract marker data
sternum_marker_data = {}
for marker in data["Markers"]:
    if marker["Name"] in sternum_markers:
        values = np.array(marker["Parts"][0]["Values"])
        sternum_marker_data[marker["Name"]] = values[:, :3]  # alleen X,Y,Z

for marker in sternum_markers:
    print(f"{marker}: {sternum_marker_data[marker].shape}")


# Bepaal het aantal frames uit een willekeurige marker
n_frames = next(iter(sternum_marker_data.values())).shape[0]

# Bouw per frame een 3xN matrix van sternum-markers (N=4 hier)
sternum_per_frame = []
for i in range(n_frames):
    frame_data = np.stack([sternum_marker_data[marker][i] for marker in sternum_markers], axis=1)  # shape (3, 4)
    sternum_per_frame.append(frame_data.T)  # shape (4, 3), later getransponeerd in r_svd naar (3, 4)



# Inlezen van trial segmentatie CSV
trials_data = pd.read_csv(csv_path)


# Analyse per trial
all_angles = []
all_translations = []

for _, row in trials_data.iterrows():
    start = int(float(row["Startframe"]))
    end = int(float(row["Eindframe"]))

    base1 = sternum_per_frame[start].T  # shape (3, 4)

    angles = []
    translations = []

    for i in range(start + 1, end + 1):
        base2 = sternum_per_frame[i].T  # shape (3, 4)
        r, t, _ = r_svd(base1, base2)
        angle_rad = np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        angles.append(angle_deg)
        translations.append(np.linalg.norm(t))

    all_angles.append(angles)
    all_translations.append(translations)


# Plot per trial
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=False)

for angles in all_angles:
    ax1.plot(angles)
ax1.set_ylabel("Rotatiehoek (°)")
ax1.set_title("Rotatie per reikbeweging")
ax1.grid()

for trans in all_translations:
    ax2.plot(trans)
ax2.set_ylabel("Translatie (mm)")
ax2.set_title("Translatie per reikbeweging")
ax2.set_xlabel("Frame index binnen trial")
ax2.grid()

fig.suptitle(f"Rotatie en translatie van het sternum tijdens reiken - {name} - {algorithm}", fontsize=16)
plt.tight_layout()
plt.show()

print("eerste plot klaar")


from collections import defaultdict

# Groeperen van rotaties en translatie per target
angles_per_target = defaultdict(list)
translations_per_target = defaultdict(list)

for _, row in trials_data.iterrows():
    start = int(row["Startframe"])
    end = int(row["Eindframe"])
    target = row["Target"]

    if start >= len(sternum_per_frame) or end >= len(sternum_per_frame):
        continue

    base1 = sternum_per_frame[start].T
    angles = []
    translations = []

    for i in range(start + 1, end + 1):
        base2 = sternum_per_frame[i].T
        r, t, _ = r_svd(base1, base2)
        angle_rad = np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        angles.append(angle_deg)
        translations.append(np.linalg.norm(t))

    angles_per_target[target].append(angles)
    translations_per_target[target].append(translations)

# Plot gemiddelden per target
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

for target in sorted(angles_per_target.keys()):
    # Padding naar gelijke lengte voor mean (NaN veilig)
    max_len = max(len(x) for x in angles_per_target[target])
    angle_mat = np.array([np.pad(x, (0, max_len - len(x)), constant_values=np.nan) for x in angles_per_target[target]])
    translation_mat = np.array([np.pad(x, (0, max_len - len(x)), constant_values=np.nan) for x in translations_per_target[target]])

    mean_angle = np.nanmean(angle_mat, axis=0)
    mean_translation = np.nanmean(translation_mat, axis=0)

    ax1.plot(mean_angle, label=target)
    ax2.plot(mean_translation, label=target)

ax1.set_ylabel("Gem. rotatiehoek (°)")
ax1.set_title("Gemiddelde rotatie per target")
ax2.set_ylabel("Gem. translatie (mm)")
ax2.set_title("Gemiddelde translatie per target")
ax2.set_xlabel("Frame index binnen trial")
ax1.legend()
ax2.legend()

plt.tight_layout()
plt.show()

print("tweede plot klaar")


lengths = [int(row["Eindframe"]) - int(row["Startframe"]) for _, row in trials_data.iterrows()]
print(f"Min: {min(lengths)}, Max: {max(lengths)}, Mean: {int(np.mean(lengths))}")


from scipy.interpolate import interp1d

def interpolate_to_fixed_length(data_list, N):
    interpolated = []
    for x in data_list:
        if len(x) < 2:
            continue  # overslaan, niet te interpoleren
        f = interp1d(np.linspace(0, 1, num=len(x)), x, kind='linear')
        x_interp = f(np.linspace(0, 1, num=N))
        interpolated.append(x_interp)
    return np.array(interpolated)

N = 100  # aantal punten per genormaliseerde trial

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

for target in sorted(angles_per_target.keys()):
    angle_interp = interpolate_to_fixed_length(angles_per_target[target], N)
    trans_interp = interpolate_to_fixed_length(translations_per_target[target], N)

    mean_angle = np.mean(angle_interp, axis=0)
    mean_trans = np.mean(trans_interp, axis=0)

    ax1.plot(mean_angle, label=target)
    ax2.plot(mean_trans, label=target)

ax1.set_ylabel("Gem. rotatiehoek (°)")
ax1.set_title("Genormaliseerde rotatie per target")
ax2.set_ylabel("Gem. translatie (mm)")
ax2.set_title("Genormaliseerde translatie per target")
ax2.set_xlabel("Genormaliseerde tijd (0–100%)")
ax1.legend()
ax2.legend()
plt.tight_layout()
plt.show()

print("derde plot klaar")



def interpolate_to_fixed_length(data_list, N):
    from scipy.interpolate import interp1d
    interpolated = []
    for x in data_list:
        if len(x) < 2:
            continue
        f = interp1d(np.linspace(0, 1, num=len(x)), x, kind='linear')
        x_interp = f(np.linspace(0, 1, num=N))
        interpolated.append(x_interp)
    return np.array(interpolated)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

for target in sorted(angles_per_target.keys()):
    angle_interp = interpolate_to_fixed_length(angles_per_target[target], N)
    trans_interp = interpolate_to_fixed_length(translations_per_target[target], N)

    for trial in angle_interp:
        ax1.plot(trial, alpha=0.5, label=target if target not in ax1.get_legend_handles_labels()[1] else "")
    for trial in trans_interp:
        ax2.plot(trial, alpha=0.5, label=target if target not in ax2.get_legend_handles_labels()[1] else "")

ax1.set_ylabel("Rotatiehoek (°)")
ax1.set_title("Alle genormaliseerde trials – rotatie")
ax2.set_ylabel("Translatie (mm)")
ax2.set_title("Alle genormaliseerde trials – translatie")
ax2.set_xlabel("Genormaliseerde tijd (0–84 punten)")
ax1.legend()
ax2.legend()

plt.tight_layout()
plt.show()

print("vierde plot klaar")

# #######################################

from collections import defaultdict
from scipy.interpolate import interp1d
import numpy as np
import pandas as pd
import json
import matplotlib.pyplot as plt

# Parameters
N_interp = 84
algorithms = ["Alg1", "Alg2"]
markers_used = ["Ster_cen", "Ster_R", "Ster_L"]

# Functies
def r_svd(base1, base2):
    base1_mean = np.mean(base1, axis=1, keepdims=True)
    base2_mean = np.mean(base2, axis=1, keepdims=True)
    base1_zero = base1 - base1_mean
    base2_zero = base2 - base2_mean
    u_arr, _, vh_arr = np.linalg.svd(base2_zero @ base1_zero.T)
    r_mat = (u_arr * np.array([1, 1, np.linalg.det(u_arr @ vh_arr)])) @ vh_arr
    trans = base2_mean - r_mat @ base1_mean
    return r_mat, trans, ((base2 - (r_mat @ base1 + trans)) ** 2).mean() ** 0.5

def interpolate_trials(trials, N):
    output = []
    for trial in trials:
        if len(trial) < 2:
            continue
        f = interp1d(np.linspace(0, 1, num=len(trial)), trial, kind='linear')
        output.append(f(np.linspace(0, 1, num=N)))
    return np.array(output)

def extract_trials_from_algorithm(subject_name, algorithm_name):

    json_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{subject_name}\\{subject_name}_{algorithm_name}.json"
    csv_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{subject_name}\\{subject_name}_trials_qualisys_{algorithm_name}.csv"

    with open(json_path, 'r') as f:
        json_data = json.load(f)

    marker_data = {}
    for marker in json_data["Markers"]:
        if marker["Name"] in markers_used:
            marker_data[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]

    n_frames = min(marker_data[m].shape[0] for m in markers_used)
    frames = [np.stack([marker_data[m][i] for m in markers_used], axis=1).T for i in range(n_frames)]

    trial_df = pd.read_csv(csv_path)
    angles_by_target = defaultdict(list)
    trans_by_target = defaultdict(list)

    for _, row in trial_df.iterrows():
        start = int(row["Startframe"])
        end = int(row["Eindframe"])
        target_label = row["Target"]
        # print(f"Start={start}, End={end}, n_frames={n_frames}")


        if start >= n_frames or end >= n_frames:
            continue

        base = frames[start].T
        angle_seq = []
        trans_seq = []
        # print(f"Toevoegen trial: {target_label}, lengte: {end - start}")

        for i in range(start + 1, end + 1):
            current = frames[i].T
            r, t, _ = r_svd(base, current)
            angle = np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))
            angle_seq.append(angle)
            trans_seq.append(np.linalg.norm(t))

        angles_by_target[target_label].append(angle_seq)
        trans_by_target[target_label].append(trans_seq)


    return angles_by_target, trans_by_target

# Stap 1: verwerk Null-data
angles_null_raw, trans_null_raw = extract_trials_from_algorithm(name, algorithm)

# Genormaliseerd gemiddelde per target
mean_angle_by_target = {}
mean_trans_by_target = {}

for target in angles_null_raw:
    angles_interp = interpolate_trials(angles_null_raw[target], N_interp)
    trans_interp = interpolate_trials(trans_null_raw[target], N_interp)
    mean_angle_by_target[target] = np.mean(angles_interp, axis=0)
    mean_trans_by_target[target] = np.mean(trans_interp, axis=0)

lengths = [int(row["Eindframe"]) - int(row["Startframe"]) for _, row in trials_data.iterrows()]
print(f"Min: {min(lengths)}, Max: {max(lengths)}, Mean: {int(np.mean(lengths))}")

# Stap 2: verwerk Alg1 en Alg2 en deel door Null-mean
for algorithm in algorithms:
    for target in sorted(mean_angle_by_target.keys()):
        fig, (ax_r, ax_t) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax_r.set_title(f"{algorithm} – Target {target} – Relatief t.o.v. Null")
        ax_r.set_ylabel("Rel. rotatie")
        ax_t.set_ylabel("Rel. translatie")
        ax_t.set_xlabel("Genormaliseerde tijd")

        angles_alg_raw, trans_alg_raw = extract_trials_from_algorithm(name, algorithm)
        angles_interp = interpolate_trials(angles_alg_raw[target], N_interp)
        trans_interp = interpolate_trials(trans_alg_raw[target], N_interp)

        for trial in angles_interp:
            rel_angle = trial / mean_angle_by_target[target]
            ax_r.plot(rel_angle, alpha=0.7)

        for trial in trans_interp:
            rel_trans = trial / mean_trans_by_target[target]
            ax_t.plot(rel_trans, alpha=0.7)

        plt.tight_layout()
        plt.show()


print("Plots voor Alg1 en Alg2 t.o.v. Null zijn gemaakt.")



for algorithm in algorithms:
    for target in sorted(mean_angle_by_target.keys()):
        fig, (ax_r, ax_t) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax_r.set_title(f"{algorithm} – Target {target} – Gemiddelde ± std t.o.v. Null")
        ax_r.set_ylabel("Rel. rotatie")
        ax_t.set_ylabel("Rel. translatie")
        ax_t.set_xlabel("Genormaliseerde tijd")

        angles_alg_raw, trans_alg_raw = extract_trials_from_algorithm(name, algorithm)
        angles_interp = interpolate_trials(angles_alg_raw[target], N_interp)
        trans_interp = interpolate_trials(trans_alg_raw[target], N_interp)

        rel_angle_matrix = angles_interp / mean_angle_by_target[target]
        rel_trans_matrix = trans_interp / mean_trans_by_target[target]

        mean_angle = np.mean(rel_angle_matrix, axis=0)
        std_angle = np.std(rel_angle_matrix, axis=0)

        mean_trans = np.mean(rel_trans_matrix, axis=0)
        std_trans = np.std(rel_trans_matrix, axis=0)

        x = np.arange(N_interp)

        ax_r.plot(x, mean_angle, label="mean")
        ax_r.fill_between(x, mean_angle - std_angle, mean_angle + std_angle, alpha=0.3)
        ax_t.plot(x, mean_trans, label="mean")
        ax_t.fill_between(x, mean_trans - std_trans, mean_trans + std_trans, alpha=0.3)

        plt.tight_layout()
        plt.show()

print("klaar met script")