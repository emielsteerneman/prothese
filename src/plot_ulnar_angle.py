### CODE IS KLAAR ###

## inladen van qualisys data
## maakt een vlak van de ulnar data
## berekent de hoek van dit vlak t.o.v. alle 3 de anatomische vlakken
## en de rotatie om de X, Y en Z-as
## en plot dit per trial
## en per target



import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

name = "AnnaZoet"
algorithm = "Null"

# === Data inladen ===
qualisys_file = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\Null.json"
with open(qualisys_file, 'r') as f:
    qualisys_data = json.load(f)

ulna_markers = ["Uln_up", "Uln_cen", "Uln_dors", "Uln_plan"]

afstand_drempel_mm = 20.0  # mm
framerate = 128.0  # Hz

def get_marker_data(name):
    for marker in qualisys_data["Markers"]:
        if marker["Name"] == name:
            return np.array(marker["Parts"][0]["Values"])[:, :3]
    raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

# === Markerdata laden ===
ulna_positions = {name: get_marker_data(name) for name in ulna_markers}
qualisys_time = np.arange(ulna_positions["Uln_up"].shape[0]) / framerate

# === Normaalvector van vlak ===
def bereken_normaalvector(p1, p2, p3):
    v1 = p2 - p1
    v2 = p3 - p1
    normaal = np.cross(v1, v2)
    return normaal / np.linalg.norm(normaal)

# === Hoeken t.o.v. anatomische vlakken ===
def hoeken_tov_assen(normaal):
    vlakken = {
        'XY': np.array([0, 0, 1]),
        'YZ': np.array([1, 0, 0]),
        'XZ': np.array([0, 1, 0])
    }
    hoeken = {}
    for vlak, referentie in vlakken.items():
        cos_theta = np.dot(normaal, referentie)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        hoek = np.degrees(np.arccos(cos_theta))
        hoeken[vlak] = hoek
    return hoeken

# === Rotaties om X, Y, Z-as ===
def rotaties_om_assen(normaal):
    rotaties = {}
    proj_YZ = normaal[[1, 2]]
    angle_x = np.degrees(np.arctan2(proj_YZ[1], proj_YZ[0]))
    rotaties['X'] = angle_x

    proj_XZ = normaal[[0, 2]]
    angle_y = np.degrees(np.arctan2(proj_XZ[1], proj_XZ[0]))
    rotaties['Y'] = angle_y

    proj_XY = normaal[[0, 1]]
    angle_z = np.degrees(np.arctan2(proj_XY[1], proj_XY[0]))
    rotaties['Z'] = angle_z

    return rotaties

# === Trialdata inlezen ===
trial_file = f"{name}_trials_qualisys_{algorithm}.csv"
trial_df = pd.read_csv(trial_file)

# === Hoeken & Rotaties per frame berekenen ===
hoek_per_frame = {vlak: [] for vlak in ['XY', 'YZ', 'XZ']}
rotatie_per_frame = {as_naam: [] for as_naam in ['X', 'Y', 'Z']}

num_frames = ulna_positions["Uln_up"].shape[0]

for i in range(num_frames):
    p1 = ulna_positions["Uln_up"][i]
    p2 = ulna_positions["Uln_dors"][i]
    p3 = ulna_positions["Uln_plan"][i]

    normaal = bereken_normaalvector(p1, p2, p3)
    hoeken = hoeken_tov_assen(normaal)
    rotaties = rotaties_om_assen(normaal)

    for vlak in hoeken:
        hoek_per_frame[vak := vlak].append(hoeken[vak])
    for as_naam in rotaties:
        rotatie_per_frame[as_naam].append(rotaties[as_naam])

# === Omzetten naar numpy-arrays ===
for key in hoek_per_frame:
    hoek_per_frame[key] = np.array(hoek_per_frame[key])
for key in rotatie_per_frame:
    rotatie_per_frame[key] = np.array(rotatie_per_frame[key])

# === Tijdverloopplots voor alle hoeken en rotaties per target ===
parameter_data = {
    "Hoek_XY": hoek_per_frame["XY"],
    "Hoek_YZ": hoek_per_frame["YZ"],
    "Hoek_XZ": hoek_per_frame["XZ"],
    "Rotatie_X": rotatie_per_frame["X"],
    "Rotatie_Y": rotatie_per_frame["Y"],
    "Rotatie_Z": rotatie_per_frame["Z"]
}

targets_uniek = trial_df["Target"].unique()
kleuren = {
    "Tar1": "dodgerblue",
    "Tar2": "orange",
    "Tar3": "mediumseagreen"
}

for param_naam, data_array in parameter_data.items():
    for target_name in targets_uniek:
        target_trials = trial_df[trial_df["Target"] == target_name]

        fig, ax = plt.subplots(figsize=(10, 4))
        fig.suptitle(f"Verloop van {param_naam.replace('_', ' ')} – Target: {target_name}", fontsize=14)

        for _, row in target_trials.iterrows():
            trial_nummer = int(row["Trial"])
            start_frame = int(row["Startframe"])
            end_frame = int(row["Eindframe"])

            trial_tijd = (np.arange(start_frame, end_frame) - start_frame) / framerate
            trial_data = data_array[start_frame:end_frame]

            ax.plot(trial_tijd, trial_data, label=f"Trial {trial_nummer}")

        ax.set_title(param_naam.replace("_", " "))
        ax.set_xlabel("Tijd binnen trial (s)")
        ax.set_ylabel("Hoek / Rotatie (°)")
        ax.grid(True)
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))

        plt.tight_layout(rect=[0, 0, 0.85, 0.95])
        plt.show()

# === Range per trial ===
range_data = []

for _, row in trial_df.iterrows():
    trial_nummer = int(row["Trial"])
    start_frame = int(row["Startframe"])
    end_frame = int(row["Eindframe"])

    entry = {"Trial": trial_nummer, "Target": row["Target"]}
    for vlak in hoek_per_frame:
        data = hoek_per_frame[vlak][start_frame:end_frame]
        entry[f"Hoek_{vlak}_range"] = np.max(data) - np.min(data)
    for as_naam in rotatie_per_frame:
        data = rotatie_per_frame[as_naam][start_frame:end_frame]
        entry[f"Rotatie_{as_naam}_range"] = np.max(data) - np.min(data)

    range_data.append(entry)

range_df = pd.DataFrame(range_data)

# === Range plotten per parameter in losse figuren ===
parameters = [col for col in range_df.columns if col not in ["Trial", "Target"]]
kleuren = {
    "Tar1": "dodgerblue",
    "Tar2": "orange",
    "Tar3": "mediumseagreen"
}

range_df = range_df.sort_values("Trial")

for param in parameters:
    fig, ax = plt.subplots(figsize=(10, 4))
    fig.suptitle(f"Range van {param.replace('_', ' ')} per trial", fontsize=14)

    for target in range_df["Target"].unique():
        df_subset = range_df[range_df["Target"] == target]
        ax.bar(df_subset["Trial"], df_subset[param], label=target, color=kleuren.get(target, 'gray'))

    ax.set_ylabel("Range (°)")
    ax.set_xlabel("Trialnummer")
    ax.set_title(param.replace("_", " "))
    ax.grid(True)
    ax.legend(title="Target")

    plt.tight_layout()
    plt.show()






















# import json
# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# name = "AnnaZoet"
# algorithm = "Null"

# # === Data inladen ===
# qualisys_file = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\Null.json"
# with open(qualisys_file, 'r') as f:
#     qualisys_data = json.load(f)

# humerus_markers = ["Hum_up", "Hum_cen", "Hum_dors", "Hum_plan"]

# afstand_drempel_mm = 20.0  # mm
# framerate = 128.0  # Hz

# def get_marker_data(name):
#     for marker in qualisys_data["Markers"]:
#         if marker["Name"] == name:
#             return np.array(marker["Parts"][0]["Values"])[:, :3]
#     raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

# # === Markerdata laden ===
# humerus_positions = {name: get_marker_data(name) for name in humerus_markers}
# qualisys_time = np.arange(humerus_positions["Hum_up"].shape[0]) / framerate

# # === Normaalvector van vlak ===
# def bereken_normaalvector(p1, p2, p3):
#     v1 = p2 - p1
#     v2 = p3 - p1
#     normaal = np.cross(v1, v2)
#     return normaal / np.linalg.norm(normaal)

# # === Hoeken t.o.v. anatomische vlakken ===
# def hoeken_tov_assen(normaal):
#     vlakken = {
#         'XY': np.array([0, 0, 1]),
#         'YZ': np.array([1, 0, 0]),
#         'XZ': np.array([0, 1, 0])
#     }
#     hoeken = {}
#     for vlak, referentie in vlakken.items():
#         cos_theta = np.dot(normaal, referentie)
#         cos_theta = np.clip(cos_theta, -1.0, 1.0)
#         hoek = np.degrees(np.arccos(cos_theta))
#         hoeken[vlak] = hoek
#     return hoeken

# # === Rotaties om X, Y, Z-as ===
# def rotaties_om_assen(normaal):
#     rotaties = {}
#     proj_YZ = normaal[[1, 2]]
#     angle_x = np.degrees(np.arctan2(proj_YZ[1], proj_YZ[0]))
#     rotaties['X'] = angle_x

#     proj_XZ = normaal[[0, 2]]
#     angle_y = np.degrees(np.arctan2(proj_XZ[1], proj_XZ[0]))
#     rotaties['Y'] = angle_y

#     proj_XY = normaal[[0, 1]]
#     angle_z = np.degrees(np.arctan2(proj_XY[1], proj_XY[0]))
#     rotaties['Z'] = angle_z

#     return rotaties

# # === Trialdata inlezen ===
# trial_file = f"{name}_trials_qualisys_{algorithm}.csv"
# trial_df = pd.read_csv(trial_file)

# # === Hoeken & Rotaties per frame berekenen ===
# hoek_per_frame = {vlak: [] for vlak in ['XY', 'YZ', 'XZ']}
# rotatie_per_frame = {as_naam: [] for as_naam in ['X', 'Y', 'Z']}

# num_frames = humerus_positions["Hum_up"].shape[0]

# for i in range(num_frames):
#     p1 = humerus_positions["Hum_up"][i]
#     p2 = humerus_positions["Hum_dors"][i]
#     p3 = humerus_positions["Hum_plan"][i]

#     normaal = bereken_normaalvector(p1, p2, p3)
#     hoeken = hoeken_tov_assen(normaal)
#     rotaties = rotaties_om_assen(normaal)

#     for vlak in hoeken:
#         hoek_per_frame[vak := vlak].append(hoeken[vak])
#     for as_naam in rotaties:
#         rotatie_per_frame[as_naam].append(rotaties[as_naam])

# # === Omzetten naar numpy-arrays ===
# for key in hoek_per_frame:
#     hoek_per_frame[key] = np.array(hoek_per_frame[key])
# for key in rotatie_per_frame:
#     rotatie_per_frame[key] = np.array(rotatie_per_frame[key])

# # === Visualisatie per target ===



# # === Tijdverloopplots voor alle hoeken en rotaties per target ===
# parameter_data = {
#     "Hoek_XY": hoek_per_frame["XY"],
#     "Hoek_YZ": hoek_per_frame["YZ"],
#     "Hoek_XZ": hoek_per_frame["XZ"],
#     "Rotatie_X": rotatie_per_frame["X"],
#     "Rotatie_Y": rotatie_per_frame["Y"],
#     "Rotatie_Z": rotatie_per_frame["Z"]
# }

# targets_uniek = trial_df["Target"].unique()
# kleuren = {
#     "Tar1": "dodgerblue",
#     "Tar2": "orange",
#     "Tar3": "mediumseagreen"
# }

# for param_naam, data_array in parameter_data.items():
#     for target_name in targets_uniek:
#         target_trials = trial_df[trial_df["Target"] == target_name]

#         fig, ax = plt.subplots(figsize=(10, 4))
#         fig.suptitle(f"Verloop van {param_naam.replace('_', ' ')} – Target: {target_name}", fontsize=14)

#         for _, row in target_trials.iterrows():
#             trial_nummer = int(row["Trial"])
#             start_frame = int(row["Startframe"])
#             end_frame = int(row["Eindframe"])

#             trial_tijd = (np.arange(start_frame, end_frame) - start_frame) / framerate
#             trial_data = data_array[start_frame:end_frame]

#             ax.plot(trial_tijd, trial_data, label=f"Trial {trial_nummer}")

#         ax.set_title(param_naam.replace("_", " "))
#         ax.set_xlabel("Tijd binnen trial (s)")
#         ax.set_ylabel("Hoek / Rotatie (°)")
#         ax.grid(True)
#         ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1))

#         plt.tight_layout(rect=[0, 0, 0.85, 0.95])
#         plt.show()

# # === Range per trial ===
# range_data = []

# for _, row in trial_df.iterrows():
#     trial_nummer = int(row["Trial"])
#     start_frame = int(row["Startframe"])
#     end_frame = int(row["Eindframe"])

#     entry = {"Trial": trial_nummer, "Target": row["Target"]}
#     for vlak in hoek_per_frame:
#         data = hoek_per_frame[vlak][start_frame:end_frame]
#         entry[f"Hoek_{vlak}_range"] = np.max(data) - np.min(data)
#     for as_naam in rotatie_per_frame:
#         data = rotatie_per_frame[as_naam][start_frame:end_frame]
#         entry[f"Rotatie_{as_naam}_range"] = np.max(data) - np.min(data)

#     range_data.append(entry)

# range_df = pd.DataFrame(range_data)

# # === Range plotten per parameter in losse figuren ===
# parameters = [col for col in range_df.columns if col not in ["Trial", "Target"]]
# kleuren = {
#     "Tar1": "dodgerblue",
#     "Tar2": "orange",
#     "Tar3": "mediumseagreen"
# }

# range_df = range_df.sort_values("Trial")

# for param in parameters:
#     fig, ax = plt.subplots(figsize=(10, 4))
#     fig.suptitle(f"Range van {param.replace('_', ' ')} per trial", fontsize=14)

#     for target in range_df["Target"].unique():
#         df_subset = range_df[range_df["Target"] == target]
#         ax.bar(df_subset["Trial"], df_subset[param], label=target, color=kleuren.get(target, 'gray'))

#     ax.set_ylabel("Range (°)")
#     ax.set_xlabel("Trialnummer")
#     ax.set_title(param.replace("_", " "))
#     ax.grid(True)
#     ax.legend(title="Target")

#     plt.tight_layout()
#     plt.show()

