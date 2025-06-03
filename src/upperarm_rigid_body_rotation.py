# import json
# from collections import defaultdict
# import numpy as np
# from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt
# import pandas as pd
# import pickle
# import os

# # --- Instellingen ---
# namenlijst = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]
# algorithms = ["Alg1", "Alg2"]
# markers_used = ["Hum_up", "Hum_plan", "Hum_dors"]
# N_interp = 100

# # --- Functies voor opslaan en laden ---
# def save_data(filename, data):
#     with open(filename, 'wb') as f:
#         pickle.dump(data, f)

# def load_data(filename):
#     if os.path.exists(filename):
#         with open(filename, 'rb') as f:
#             return pickle.load(f)
#     return None

# # --- Gegeven functies ---
# def r_svd(base1, base2):
#     base1_mean = np.mean(base1, axis=1, keepdims=True)
#     base2_mean = np.mean(base2, axis=1, keepdims=True)
#     base1_zero = base1 - base1_mean
#     base2_zero = base2 - base2_mean
#     u, _, vh = np.linalg.svd(base2_zero @ base1_zero.T)
#     r = (u * np.array([1, 1, np.linalg.det(u @ vh)])) @ vh
#     t = base2_mean - r @ base1_mean
#     return r, t, ((base2 - (r @ base1 + t)) ** 2).mean() ** 0.5

# def interpolate_trials(trials, N):
#     output = []
#     for trial in trials:
#         if len(trial) < 2:
#             continue
#         f = interp1d(np.linspace(0, 1, num=len(trial)), trial, kind='linear')
#         output.append(f(np.linspace(0, 1, num=N)))
#     return np.array(output)

# def extract_trials(name, algorithm):
#     json_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
#     csv_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_{algorithm}.csv"
    
#     with open(json_path, 'r') as f:
#         json_data = json.load(f)

#     marker_data = {}
#     for marker in json_data["Markers"]:
#         if marker["Name"] in markers_used:
#             marker_data[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]

#     n_frames = min(marker_data[m].shape[0] for m in markers_used)
#     frames = [np.stack([marker_data[m][i] for m in markers_used], axis=1).T for i in range(n_frames)]

#     trial_df = pd.read_csv(csv_path)
#     trial_df.columns = trial_df.columns.str.strip()

#     angles = defaultdict(list)
#     trans = defaultdict(list)

#     for _, row in trial_df.iterrows():
#         start = int(float(row["Startframe"]))
#         end = int(float(row["Eindframe"]))
#         target = row["Target"]

#         if start >= n_frames or end >= n_frames:
#             continue

#         base = frames[start].T
#         angle_seq = []
#         trans_seq = []

#         for i in range(start + 1, end + 1):
#             current = frames[i].T
#             r, t, _ = r_svd(base, current)
#             angle = np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))
#             angle_seq.append(angle)
#             trans_seq.append(np.linalg.norm(t))

#         angles[target].append(angle_seq)
#         trans[target].append(trans_seq)

#     return angles, trans


# # --- Data verwerking en caching ---
# data_filename = "processed_data_upper_arm.pkl"
# saved = load_data(data_filename)

# if saved:
#     pooled_angles, pooled_trans = saved
#     print("Gelaadde gepoolde data uit opgeslagen bestand.")
# else:
#     pooled_angles = defaultdict(list)
#     pooled_trans = defaultdict(list)

#     for idx, name in enumerate(namenlijst, 1):
#         print(f"[{idx}/{len(namenlijst)}] Verwerken van {name} begonnen...")

#         angles_null, trans_null = extract_trials(name, "Null")
#         mean_angle_null = {k: np.mean(interpolate_trials(v, N_interp), axis=0) for k, v in angles_null.items()}
#         mean_trans_null = {k: np.mean(interpolate_trials(v, N_interp), axis=0) for k, v in trans_null.items()}

#         for algorithm in algorithms:
#             angles_alg, trans_alg = extract_trials(name, algorithm)
#             for target in angles_alg:
#                 if target not in mean_angle_null or target not in mean_trans_null:
#                     continue
#                 angles_interp = interpolate_trials(angles_alg[target], N_interp)
#                 trans_interp = interpolate_trials(trans_alg[target], N_interp)

#                 rel_angle = angles_interp / np.interp(
#                     np.linspace(0, 1, N_interp),
#                     np.linspace(0, 1, len(mean_angle_null[target])),
#                     mean_angle_null[target]
#                 )
#                 rel_trans = trans_interp / mean_trans_null[target]

#                 pooled_angles[(algorithm, target)].extend(rel_angle)
#                 pooled_trans[(algorithm, target)].extend(rel_trans)

#         print(f"Alle null data verwerkt voor {name}")

#     save_data(data_filename, (pooled_angles, pooled_trans))
#     print("Data opgeslagen voor toekomstig gebruik.")

# # --- Plotten absolute waarden per trial voor NULL meting ---
# print("\nPlotten van absolute waarden voor NULL meting...")

# for target in sorted(angles_null.keys()):
#     angles_interp = interpolate_trials(angles_null[target], N_interp)
#     trans_interp = interpolate_trials(trans_null[target], N_interp)

#     x = np.arange(N_interp)
#     fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
#     fig.suptitle(f"Null – {target} – Absolute waarden ({name})")

#     # Rotatie
#     for i, trial in enumerate(angles_interp):
#         ax1.plot(x, trial, label=f"Trial {i+1}")
#     ax1.set_ylabel("Rotatie (graden)")
#     ax1.legend(loc='upper right', fontsize='small')
#     ax1.set_ylim(-10, 50)

#     # Translatie
#     for i, trial in enumerate(trans_interp):
#         ax2.plot(x, trial, label=f"Trial {i+1}")
#     ax2.set_ylabel("Translatie (mm)")
#     ax2.set_xlabel("Genormaliseerde tijd")
#     ax2.set_ylim(-20, 200)

#     plt.tight_layout()
#     plt.show()


# # --- Plotten absolute waarden per trial vóór normalisatie ---
# print("\nPlotten van absolute waarden per trial vóór normalisatie...")

# for algorithm in algorithms:
#     for name in namenlijst:
#         angles_alg, trans_alg = extract_trials(name, algorithm)
#         for target in sorted(angles_alg.keys()):
#             angles_interp = interpolate_trials(angles_alg[target], N_interp)
#             trans_interp = interpolate_trials(trans_alg[target], N_interp)

#             x = np.arange(N_interp)
#             fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
#             fig.suptitle(f"{algorithm} – {target} – Absolute waarden")  #  ({name})

#             # Plot rotaties
#             for i, trial in enumerate(angles_interp):
#                 ax1.plot(x, trial, label=f"Trial {i+1}")
#             ax1.set_ylabel("Rotatie (graden)")
#             ax1.legend(loc='upper right', fontsize='small')
#             ax1.set_ylim(-10, 50)

#             # Plot translaties
#             for i, trial in enumerate(trans_interp):
#                 ax2.plot(x, trial, label=f"Trial {i+1}")
#             ax2.set_ylabel("Translatie (mm)")
#             ax2.set_xlabel("Genormaliseerde tijd")
#             ax2.set_ylim(-20, 200)

#             plt.tight_layout()
#             plt.show()


# # --- Plotten genormaliseerde gepoolde data ---
# print("\nPlotten van genormaliseerde gepoolde data...")
# target = ["Tar1", "Tar2", "Tar3"]
# algorithm = ["Alg1", "Alg2"]

# kleur_per_algoritme = {
#     "Alg1": "royalblue",       # None betekent: gebruik standaard matplotlib kleur
#     "Alg2": "green"
# }

# for algorithm in algorithms:
#     kleur = kleur_per_algoritme.get(algorithm, "black")  # fallback zwart

#     for target in sorted(set(k[1] for k in pooled_angles.keys())):
#         angle_mat = np.vstack(pooled_angles[(algorithm, target)])
#         trans_mat = np.vstack(pooled_trans[(algorithm, target)])
#         mean_angle = np.mean(angle_mat, axis=0)
#         std_angle = np.std(angle_mat, axis=0)
#         mean_trans = np.mean(trans_mat, axis=0)
#         std_trans = np.std(trans_mat, axis=0)

#         target_label = target.replace("Tar", "")
#         algorithm_label = algorithm.replace("Alg", "Algorithm ")

#         print(f"Plotten: {algorithm} – Target {target}")

#         x = np.arange(N_interp)
#         fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
#         ax1.set_title(f"Mean Normalized Upper Arm Rotation and Translation ± SD - {algorithm_label} - Target {target_label}", fontsize=20)
#         ax1.plot(x, mean_angle, label="Mean", color=kleur)
#         ax1.fill_between(x, mean_angle - std_angle, mean_angle + std_angle, alpha=0.3, color=kleur)
#         ax1.set_ylabel("Relative rotation", fontsize=18)
#         ax1.tick_params(axis='y', labelsize=16)
#         ax1.set_ylim(-10, 40)
#         ax1.set_xlim(0, N_interp - 1)
#         ax1.set_xticks(np.arange(0, N_interp, 10))



#         ax2.plot(x, mean_trans, label="Mean", color=kleur)
#         ax2.fill_between(x, mean_trans - std_trans, mean_trans + std_trans, alpha=0.3, color=kleur)
#         ax2.set_ylabel("Relative translation", fontsize=18)
#         ax2.set_xlabel("Normalized time", fontsize=18)
#         ax2.tick_params(axis='x', labelsize=16)
#         ax2.tick_params(axis='y', labelsize=16)
#         ax2.set_ylim(-10, 40)
#         ax2.set_xlim(0, N_interp - 1)
#         ax2.set_xticks(np.arange(0, N_interp, 10))



#         plt.tight_layout()
#         plt.show()

# print("Klaar met het script.")

import json
from collections import defaultdict
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import pandas as pd
import pickle
import os

# --- Instellingen ---
namenlijst = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]
algorithms = ["Alg1", "Alg2"]
markers_used = ["Hum_up", "Hum_plan", "Hum_dors"]
N_interp = 100

# --- Functies voor opslaan en laden ---
def nested_defaultdict_list():
    return defaultdict(list)

def save_data(filename, data):
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

def load_data(filename):
    if os.path.exists(filename):
        with open(filename, 'rb') as f:
            return pickle.load(f)
    return None

# --- Gegeven functies ---
def r_svd(base1, base2):
    base1_mean = np.mean(base1, axis=1, keepdims=True)
    base2_mean = np.mean(base2, axis=1, keepdims=True)
    base1_zero = base1 - base1_mean
    base2_zero = base2 - base2_mean
    u, _, vh = np.linalg.svd(base2_zero @ base1_zero.T)
    r = (u * np.array([1, 1, np.linalg.det(u @ vh)])) @ vh
    t = base2_mean - r @ base1_mean
    return r, t, ((base2 - (r @ base1 + t)) ** 2).mean() ** 0.5

def interpolate_trials(trials, N):
    output = []
    for trial in trials:
        if len(trial) < 2:
            continue
        f = interp1d(np.linspace(0, 1, num=len(trial)), trial, kind='linear')
        output.append(f(np.linspace(0, 1, num=N)))
    return np.array(output)

def extract_trials(name, algorithm):
    json_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
    csv_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_{algorithm}.csv"
    
    with open(json_path, 'r') as f:
        json_data = json.load(f)

    marker_data = {}
    for marker in json_data["Markers"]:
        if marker["Name"] in markers_used:
            marker_data[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]

    n_frames = min(marker_data[m].shape[0] for m in markers_used)
    frames = [np.stack([marker_data[m][i] for m in markers_used], axis=1).T for i in range(n_frames)]

    trial_df = pd.read_csv(csv_path)
    trial_df.columns = trial_df.columns.str.strip()

    angles = defaultdict(list)
    trans = defaultdict(list)

    for _, row in trial_df.iterrows():
        start = int(float(row["Startframe"]))
        end = int(float(row["Eindframe"]))
        target = row["Target"]

        if start >= n_frames or end >= n_frames:
            continue

        base = frames[start].T
        angle_seq = []
        trans_seq = []

        for i in range(start + 1, end + 1):
            current = frames[i].T
            r, t, _ = r_svd(base, current)
            angle = np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))
            angle_seq.append(angle)
            trans_seq.append(np.linalg.norm(t))

        angles[target].append(angle_seq)
        trans[target].append(trans_seq)

    return angles, trans

# --- Data verwerking en caching ---
data_filename = "processed_data_upper_arm_with_max.pkl"
saved = load_data(data_filename)

if saved:
    pooled_angles, pooled_trans, max_vals_angle_per_trial, max_vals_trans_per_trial = saved
    print("Gelaadde gepoolde data uit opgeslagen bestand.")
else:
    pooled_angles = defaultdict(list)
    pooled_trans = defaultdict(list)

    max_vals_angle_per_trial = defaultdict(nested_defaultdict_list)
    max_vals_trans_per_trial = defaultdict(nested_defaultdict_list)


    for idx, name in enumerate(namenlijst, 1):
        print(f"[{idx}/{len(namenlijst)}] Verwerken van {name} begonnen...")

        angles_null, trans_null = extract_trials(name, "Null")
        mean_angle_null = {k: np.mean(interpolate_trials(v, N_interp), axis=0) for k, v in angles_null.items()}
        mean_trans_null = {k: np.mean(interpolate_trials(v, N_interp), axis=0) for k, v in trans_null.items()}

        for algorithm in algorithms:
            angles_alg, trans_alg = extract_trials(name, algorithm)
            for target in angles_alg:
                if target not in mean_angle_null or target not in mean_trans_null:
                    continue
                angles_interp = interpolate_trials(angles_alg[target], N_interp)
                trans_interp = interpolate_trials(trans_alg[target], N_interp)

                rel_angle = angles_interp / np.interp(
                    np.linspace(0, 1, N_interp),
                    np.linspace(0, 1, len(mean_angle_null[target])),
                    mean_angle_null[target]
                )
                rel_trans = trans_interp / mean_trans_null[target]

                pooled_angles[(algorithm, target)].extend(rel_angle)
                pooled_trans[(algorithm, target)].extend(rel_trans)

                for i in range(len(rel_angle)):
                    max_vals_angle_per_trial[(algorithm, target)][i].append(np.max(rel_angle[i]))
                    max_vals_trans_per_trial[(algorithm, target)][i].append(np.max(rel_trans[i]))

        print(f"Alle null data verwerkt voor {name}")

    save_data(data_filename, (pooled_angles, pooled_trans, max_vals_angle_per_trial, max_vals_trans_per_trial))
    print("Data opgeslagen voor toekomstig gebruik.")

# --- Plotten barplots genormaliseerde max waarden per trial ---
kleur_per_algoritme = {
    "Alg1": "royalblue",
    "Alg2": "seagreen"
}

print("\nBarplots maken voor genormaliseerde maximale waarden per trial...")

target = ["Tar1", "Tar2", "Tar3"]
algorithm = ["Alg1", "Alg2"]

# for algorithm in algorithms:
#     kleur = kleur_per_algoritme.get(algorithm, "gray")

#     for target in sorted(set(k[1] for k in max_vals_angle_per_trial.keys())):
#         angle_data = max_vals_angle_per_trial[(algorithm, target)]
#         trans_data = max_vals_trans_per_trial[(algorithm, target)]

#         trials = sorted(angle_data.keys())
#         mean_angles = [np.mean(angle_data[t]) for t in trials]
#         std_angles = [np.std(angle_data[t]) for t in trials]
#         mean_trans = [np.mean(trans_data[t]) for t in trials]
#         std_trans = [np.std(trans_data[t]) for t in trials]
#         mean_angles = mean_angles[:10]
#         std_angles = std_angles[:10]
#         mean_trans = mean_trans[:10]
#         std_trans = std_trans[:10]
#         trials = trials[:10]
#         target_label = target.replace("Tar", "")
#         algorithm_label = algorithm.replace("Alg", "Algorithm ")

#         x = np.arange(len(mean_trans))
#         width = 0.6

#         fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
#         fig.suptitle(f"Mean Normalized Maximum Upper Arm Rotation and Translation ± SD - {algorithm_label} - Target {target_label}", fontsize=20)

#         # Bar chart rotatie
#         ax1.bar(x, mean_angles, yerr=std_angles, capsize=5, color=kleur)
#         ax1.set_ylabel("Maximum rotation", fontsize=18)
#         # ax1.set_ylim(0, max(mean_angles + std_angles) * 1.3)
#         ax1.set_ylim(0,40)
#         ax1.set_yticks(np.arange(0, 41, 10))#, fontsize=16)
#         ax1.set_yticklabels(np.arange(0, 41, 10), fontsize=16)

#         # Bar chart translatie
#         ax2.bar(x, mean_trans, yerr=std_trans, capsize=5, color=kleur)
#         ax2.set_ylabel("Maximum translation", fontsize=18)
#         ax2.set_xlabel("Trial number", fontsize=18)
#         ax2.set_xticks(x)
#         ax2.set_xticklabels([str(i+1) for i in trials], fontsize=16)
#         # ax2.set_ylim(0, max(mean_trans + std_trans) * 1.3)
#         ax2.set_ylim(0,80)
#         ax2.set_yticks(np.arange(0, 81, 20))#, fontsize=16)
#         ax2.set_yticklabels(np.arange(0, 81, 20), fontsize=16)


#         plt.tight_layout()
#         plt.show()


# for algorithm in algorithms:
#     kleur = kleur_per_algoritme.get(algorithm, "black")  # fallback zwart

#     for target in sorted(set(k[1] for k in pooled_angles.keys())):
#         angle_mat = np.vstack(pooled_angles[(algorithm, target)])
#         trans_mat = np.vstack(pooled_trans[(algorithm, target)])
#         mean_angle = np.mean(angle_mat, axis=0)
#         std_angle = np.std(angle_mat, axis=0)
#         mean_trans = np.mean(trans_mat, axis=0)
#         std_trans = np.std(trans_mat, axis=0)

#         target_label = target.replace("Tar", "")
#         algorithm_label = algorithm.replace("Alg", "Algorithm ")

#         print(f"Plotten: {algorithm} – Target {target}")

#         x = np.arange(N_interp)
#         fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
#         ax1.set_title(f"Mean Normalized Upper Arm Rotation and Translation ± SD - {algorithm_label} - Target {target_label}", fontsize=20)
#         ax1.plot(x, mean_angle, label="Mean", color=kleur)
#         ax1.fill_between(x, mean_angle - std_angle, mean_angle + std_angle, alpha=0.3, color=kleur)
#         ax1.set_ylabel("Rotation", fontsize=18)
#         ax1.tick_params(axis='y', labelsize=16)
#         ax1.set_ylim(-10, 40)
#         ax1.set_xlim(0, N_interp - 1)
#         ax1.set_xticks(np.arange(0, N_interp, 10))



#         ax2.plot(x, mean_trans, label="Mean", color=kleur)
#         ax2.fill_between(x, mean_trans - std_trans, mean_trans + std_trans, alpha=0.3, color=kleur)
#         ax2.set_ylabel("Translation", fontsize=18)
#         ax2.set_xlabel("Time", fontsize=18)
#         ax2.tick_params(axis='x', labelsize=16)
#         ax2.tick_params(axis='y', labelsize=16)
#         ax2.set_ylim(-10, 60)
#         # ax2.set_yticks(np.arange(0, 61, 10))#, fontsize=16)
#         ax2.set_xlim(0, N_interp - 1)
#         ax2.set_xticks(np.arange(0, N_interp, 10))



#         plt.tight_layout()
#         plt.show()

import numpy as np
import matplotlib.pyplot as plt

# Tinten per algoritme voor de 3 targets
kleuren_per_target = {
    "Alg1": ["#4169E1", "#5A9BD4", "#B0C4DE"],  # Blauwtinten
    "Alg2": ["#2E8B57", "#66CDAA", "#98FB98"],  # Groentinten
}

bar_width = 0.25
trials_shown = 10

for algorithm in algorithms:
    kleur_list = kleuren_per_target.get(algorithm, ["gray", "gray", "gray"])
    targets = sorted(set(k[1] for k in max_vals_angle_per_trial.keys() if k[0] == algorithm))
    
    x = np.arange(trials_shown)  # trial 1 t/m 10
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Mean Normalized Maximum Upper Arm Rotation and Translation ± SD - {algorithm.replace('Alg', 'Algorithm ')}", fontsize=20)

    for idx, target in enumerate(targets):
        angle_data = max_vals_angle_per_trial[(algorithm, target)]
        trans_data = max_vals_trans_per_trial[(algorithm, target)]

        trials = sorted(angle_data.keys())[:trials_shown]
        mean_angles = [np.mean(angle_data[t]) for t in trials]
        std_angles = [np.std(angle_data[t]) for t in trials]
        mean_trans = [np.mean(trans_data[t]) for t in trials]
        std_trans = [np.std(trans_data[t]) for t in trials]

        kleur = kleur_list[idx]
        offset = (idx - 1) * bar_width  # -1, 0, 1 voor 3 targets

        # Rotatie subplot
        ax1.bar(x + offset, mean_angles, width=bar_width, yerr=std_angles, capsize=5, label=target.replace("Tar", "Target "), color=kleur)

        # Translatie subplot
        ax2.bar(x + offset, mean_trans, width=bar_width, yerr=std_trans, capsize=5, label=target.replace("Tar", "Target "), color=kleur)

    # Rotatie instellingen
    ax1.set_ylabel("Maximum rotation", fontsize=16)
    ax1.set_ylim(0, 40)
    ax1.set_yticks(np.arange(0, 41, 10))
    ax1.tick_params(axis='y', labelsize=14)
    ax1.legend(loc='upper right', bbox_to_anchor=(1, 1), fontsize=12)

    # Translatie instellingen
    ax2.set_ylabel("Maximum translation", fontsize=16)
    ax2.set_xlabel("Trial number", fontsize=16)
    ax2.set_ylim(0, 80)
    ax2.set_yticks(np.arange(0, 81, 20))
    ax2.set_xticks(x)
    ax2.set_xticklabels([str(i+1) for i in x], fontsize=14)
    ax2.tick_params(axis='y', labelsize=14)
    ax2.legend(loc='upper right', bbox_to_anchor=(1, 1), fontsize=12)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # ruimte voor titel
    plt.show()

