### CODE IS KLAAR ###

## inladen van qualisys data
## maakt een vlak van de sternum data
## berekent de hoek van dit vlak t.o.v. het frontale vlak
## en de rotatie om de Y-as
## en plot dit per trial
## en per target

import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

name = "MartZoet"
algorithm = "Alg1"

## qualisys file
qualisys_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
with open(qualisys_file, 'r') as f:
    qualisys_data = json.load(f)
finger_marker = "Index"
target_markers = ["Tar1", "Tar2", "Tar3"]
sternum_markers = ["Ster_cen", "Ster_L", "Ster_R"]

framerate = 128.0  # Hz

## marker data uitlezen
def get_marker_data(name):
    for marker in qualisys_data["Markers"]:
        if marker["Name"] == name:
            return np.array(marker["Parts"][0]["Values"])[:, :3]
    raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

finger_pos = get_marker_data(finger_marker)
targets = {name: get_marker_data(name) for name in target_markers}
sternum_positions = {name: get_marker_data(name) for name in sternum_markers}
qualisys_time = np.arange(finger_pos.shape[0]) / framerate



# === Hoek t.o.v. YZ-vlak ===
def bereken_hoek_van_vlak(ster_L, ster_R, ster_cen):
    v1 = ster_R - ster_L
    v2 = ster_cen - ster_L
    normaal = np.cross(v1, v2)
    normaal = normaal / np.linalg.norm(normaal)

    yz_normaal = np.array([-1, 0, 0])  # AANGEPAST: als X-as naar achter wijst
    cos_theta = np.dot(normaal, yz_normaal)
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    hoek_in_graden = np.degrees(theta)
    return hoek_in_graden


# === Rotatie om Y-as ===
def bereken_rotatie_om_y_as(ster_L, ster_R, ster_cen):
    v1 = ster_R - ster_L
    v2 = ster_cen - ster_L
    normaal = np.cross(v1, v2)
    normaal = normaal / np.linalg.norm(normaal)

    # Projecteer de normaalvector op het XZ-vlak
    normaal_proj = np.array([normaal[0], 0, normaal[2]])
    normaal_proj = normaal_proj / np.linalg.norm(normaal_proj)

    # Hoek met de Z-as (X-as wordt de referentie voor rotatie)
    z_as = np.array([0, 0, 1])
    cos_theta = np.dot(normaal_proj, z_as)
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    hoek_rotatie_om_y = np.degrees(theta)

    # Bepaal of de rotatie linksom of rechtsom is
    richting = np.cross(z_as, normaal_proj)[1]  # Gebruik de Y-component van het kruisproduct
    if richting < 0:
        hoek_rotatie_om_y = -hoek_rotatie_om_y

    return hoek_rotatie_om_y




# === 1. Lees het CSV-bestand met trialinformatie in ===
trial_file = f"{name}_trials_qualisys_{algorithm}.csv"
trial_df = pd.read_csv(trial_file)

# === 2. Bereken per frame de sternum-hoek en -rotatie ===
hoek_per_frame = []
rotatie_per_frame = []

for i in range(finger_pos.shape[0]):
    ster_L = sternum_positions["Ster_L"][i]
    ster_R = sternum_positions["Ster_R"][i]
    ster_cen = sternum_positions["Ster_cen"][i]

    hoek = bereken_hoek_van_vlak(ster_L, ster_R, ster_cen)
    rotatie = bereken_rotatie_om_y_as(ster_L, ster_R, ster_cen)

    hoek_per_frame.append(hoek)
    rotatie_per_frame.append(rotatie)

hoek_per_frame = np.array(hoek_per_frame)
rotatie_per_frame = np.array(rotatie_per_frame)
# Groepeer trials per target
targets_uniek = trial_df["Target"].unique()

for target_name in targets_uniek:
    # Filter op huidige target
    target_trials = trial_df[trial_df["Target"] == target_name]

    # Maak figuur met 2 subplots: sternumhoek & sternumrotatie
    fig, axs = plt.subplots(2, 1, sharex=False)
    fig.suptitle(f"Verloop sternumhoek en rotatie – Target: {target_name}", fontsize=14)

    for _, row in target_trials.iterrows():
        trial_nummer = int(row["Trial"])
        start_frame = int(row["Startframe"])
        end_frame = int(row["Eindframe"])

        # Tijd-as binnen trial
        trial_tijd = (np.arange(start_frame, end_frame) - start_frame) / framerate

        # Extract data
        trial_hoeken = hoek_per_frame[start_frame:end_frame]
        trial_rotaties = rotatie_per_frame[start_frame:end_frame]

        # Plot sternumhoek
        axs[0].plot(trial_tijd, trial_hoeken, label=f"Trial {trial_nummer}")
        # Plot rotatie
        axs[1].plot(trial_tijd, trial_rotaties, label=f"Trial {trial_nummer}")

    # Opmaak subplots
    axs[0].set_title("Sternumhoek t.o.v. YZ-vlak")
    axs[0].set_ylabel("Hoek (graden)")
    axs[0].grid(True)

    axs[1].set_title("Sternumrotatie om Y-as")
    axs[1].set_xlabel("Tijd binnen trial (s)")
    axs[1].set_ylabel("Rotatie (graden)")
    axs[1].grid(True)

    axs[0].legend(loc='upper left', bbox_to_anchor=(1.05, 1))
    # plt.tight_layout(rect=[0, 0, 0.85, 0.95])
    plt.show()




# === 3. Bereken range per trial ===
range_data = []

for _, row in trial_df.iterrows():
    trial_nummer = int(row["Trial"])
    start_frame = int(row["Startframe"])
    end_frame = int(row["Eindframe"])

    trial_hoeken = hoek_per_frame[start_frame:end_frame]
    trial_rotaties = rotatie_per_frame[start_frame:end_frame]

    hoek_range = np.max(trial_hoeken) - np.min(trial_hoeken)
    rotatie_range = np.max(trial_rotaties) - np.min(trial_rotaties)

    range_data.append({
        "Trial": trial_nummer,
        "Target": row["Target"],
        "Hoek_range": hoek_range,
        "Rotatie_range": rotatie_range
    })

range_df = pd.DataFrame(range_data)

# === 4. Plot de ranges als staafdiagrammen per target met kleuren ===
fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
fig.suptitle("Range van sternumhoek en -rotatie per trial", fontsize=14)

# Kleurtoewijzing per target
kleuren = {
    "Tar1": "dodgerblue",
    "Tar2": "orange",
    "Tar3": "mediumseagreen"
}

# Sorteer op trialnummer
range_df = range_df.sort_values("Trial")

# Subplot 1: Hoek-range
for target in range_df["Target"].unique():
    df_subset = range_df[range_df["Target"] == target]
    axs[0].bar(df_subset["Trial"], df_subset["Hoek_range"], 
               label=target, color=kleuren[target])

axs[0].set_ylabel("Range hoek (°)")
axs[0].set_title("Sternumhoek-range per trial")
axs[0].grid(True)
axs[0].legend(title="Target")

# Subplot 2: Rotatie-range
for target in range_df["Target"].unique():
    df_subset = range_df[range_df["Target"] == target]
    axs[1].bar(df_subset["Trial"], df_subset["Rotatie_range"], 
               label=target, color=kleuren[target])

axs[1].set_ylabel("Range rotatie (°)")
axs[1].set_xlabel("Trialnummer")
axs[1].set_title("Sternumrotatie-range per trial")
axs[1].grid(True)
axs[1].legend(title="Target")

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()
