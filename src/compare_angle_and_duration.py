### dit bestand is waardeloos en moet aangepast worden om het echt te kunnen gebruiken ###


import json
import numpy as np
import matplotlib.pyplot as plt

# === Instellingen ===
json_paths = [
    "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\Null.json",
    "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\Alg1.json",
    "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\Alg2.json"
]
vinger_marker = "Index"
target_markers = ["Tar1", "Tar2", "Tar3"]
afstand_drempel_mm = 20.0
y_start_drempel_boven_min_mm = 33.0
framerate = 128.0  # Hz

# === JSON inladen ===
def load_json(file_path):
    with open(file_path, "r") as f:
        return json.load(f)

# === Markerdata extraheren ===
def get_marker_data(data, name):
    for marker in data["Markers"]:
        if marker["Name"] == name:
            return np.array(marker["Parts"][0]["Values"])[:, :3]
    raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

# === Detecteer reiken op basis van Y ===
def detect_reiken_op_basis_van_y(finger_pos, target_pos, afstand_drempel, y_drempel_boven_min, fps):
    afstanden = np.linalg.norm(finger_pos - target_pos, axis=1)
    binnen = afstanden < afstand_drempel
    raak_frames = np.where(np.diff(binnen.astype(int)) == 1)[0] + 1

    y_min = np.min(finger_pos[:, 1])
    y_drempel = y_min + y_drempel_boven_min

    pogingen = []
    laatste_start_tijd = -np.inf

    for raak_frame in raak_frames:
        voor_moment = np.where(finger_pos[:raak_frame, 1] < y_drempel)[0]
        if len(voor_moment) == 0:
            continue
        start_frame = voor_moment[-1]
        duur = (raak_frame - start_frame) / fps
        start_tijd = start_frame / fps
        raak_tijd = raak_frame / fps

        if start_tijd - laatste_start_tijd >= 1.0:
            pogingen.append((start_frame, raak_frame, start_tijd, raak_tijd, duur))
            laatste_start_tijd = start_tijd

    return pogingen

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

    # Bereken de rotatiehoek om de Y-as (hoek tussen normaalvector en geprojecteerde vector)
    cos_phi = np.dot(normaal, normaal_proj)
    phi = np.arccos(np.clip(cos_phi, -1.0, 1.0))
    phi_in_graden = np.degrees(phi)
    return phi_in_graden

# === Vergelijking van de data ===
def vergelijk_data(data_files):
    # Laad de data uit de bestanden
    loaded_data = [load_json(file) for file in data_files]

    # Opslaan van pogingen per bestand
    pogingen_per_bestand = {}

    # Itereer over de bestanden en verzamel de pogingen en statistieken
    for file_idx, data in enumerate(loaded_data):
        print(f"\n🎯 Bestandsresultaten voor: {data_files[file_idx]}")
        finger_pos = get_marker_data(data, vinger_marker)
        targets = {name: get_marker_data(data, name) for name in target_markers}
        sternum_positions = {name: get_marker_data(data, name) for name in ["Ster_cen", "Ster_L", "Ster_R"]}

        pogingen_per_bestand[file_idx] = {}

        for target_name in target_markers:
            target_pos = targets[target_name]
            pogingen = detect_reiken_op_basis_van_y(finger_pos, target_pos, afstand_drempel_mm, y_start_drempel_boven_min_mm, framerate)
            pogingen_per_bestand[file_idx][target_name] = []

            # Verzamel pogingen en statistieken
            for i, (start, raak, t_start, t_raak, duur) in enumerate(pogingen, 1):
                frames = np.arange(start, raak + 1)
                tijden = (frames - start) / framerate
                hoeken = []
                rotaties = []

                for frame in frames:
                    ster_L = sternum_positions["Ster_L"][frame]
                    ster_R = sternum_positions["Ster_R"][frame]
                    ster_cen = sternum_positions["Ster_cen"][frame]
                    hoeken.append(bereken_hoek_van_vlak(ster_L, ster_R, ster_cen))
                    rotaties.append(bereken_rotatie_om_y_as(ster_L, ster_R, ster_cen))

                hoeken = np.array(hoeken)
                rotaties = np.array(rotaties)

                # Opslaan van de gegevens
                pogingen_per_bestand[file_idx][target_name].append({
                    'tijden': tijden,
                    'hoeken': hoeken,
                    'rotaties': rotaties,
                    'duurtijd': duur
                })

    return pogingen_per_bestand

# === Visualisatie ===
def plot_comparisons(pogingen_per_bestand):
    fig, axs = plt.subplots(len(target_markers), 1, figsize=(10, 4 * len(target_markers)))
    fig.suptitle("📈 Vergelijking van Hoeken en Rotaties voor verschillende Bestanden", fontsize=16)

    if len(target_markers) == 1:
        axs = [axs]

    for idx, target_name in enumerate(target_markers):
        ax = axs[idx]
        print(f"\n🎯 Vergelijking voor Target: {target_name}")

        # Vergelijk de pogingen voor elk bestand
        for file_idx, file_name in enumerate(pogingen_per_bestand):
            for poging in pogingen_per_bestand[file_idx][target_name]:
                ax.plot(poging['tijden'], poging['hoeken'], label=f"{file_name} Hoek", linewidth=2)
                ax.plot(poging['tijden'], poging['rotaties'], label=f"{file_name} Rotatie", linestyle='--', linewidth=2)

                # Print statistieken
                print(f"Bestand {file_name} | Duur: {poging['duurtijd']:.2f}s")

        ax.set_title(f"🎯 Target: {target_name}")
        ax.set_xlabel("Tijd sinds start (s)")
        ax.set_ylabel("Hoek (°) / Rotatie (°)")
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

# === Start vergelijking ===
pogingen_per_bestand = vergelijk_data(json_paths)
plot_comparisons(pogingen_per_bestand)
