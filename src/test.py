import json
import numpy as np
import pandas as pd
from collections import defaultdict
from time import sleep

namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", 
        "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]

# Standaard markers voor iedereen
default_markers_used = ["Hum_cen", "Hum_dors", "Hum_up"] #dors niet de issue, cen niet de issue, up niet

# Override voor specifieke proefpersonen (indien nodig)
markers_per_person = {
    # "AnnaZoet": ["Ster_cen", "Ster_R", "Ster_L"],
    # voeg hier andere uitzonderingen toe als nodig
}


def euclid_distance(base1, base2):
    base1_mean = np.mean(base1, axis=1, keepdims=True)
    base2_mean = np.mean(base2, axis=1, keepdims=True)
    # base1_zero = base1 - base1_mean
    # base2_zero = base2 - base2_mean
    distance = np.linalg.norm(base1_mean - base2_mean, axis=1)
    return distance




for naam in namen:
    print(f"\n====================\nAnalyse voor {naam}:\n====================")
    
    # Null data
    marker_data_path_null = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{naam}\\{naam}_Null.json"
    trial_file_null = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Null.csv")

    with open(marker_data_path_null, 'r') as f:
        marker_data_json_null = json.load(f)
    marker_data_null = {}

    # Kies markers voor deze persoon
    markers_used = markers_per_person.get(naam, default_markers_used)

    for marker in marker_data_json_null["Markers"]:
        if marker["Name"] in markers_used:
            marker_data_null[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]
    
    n_frames_null = min(marker_data_null[m].shape[0] for m in markers_used)
    frames_null = [np.stack([marker_data_null[m][i] for m in markers_used], axis=1).T for i in range(n_frames_null)]

    # max_rotation_dict_null = defaultdict(list)
    max_translation_dict_null = defaultdict(list)

    for _, row in trial_file_null.iterrows():
        start_null = int(float(row["Startframe"]))
        end_null = int(float(row["Eindframe"]))
        target_null = row["Target"]

        if start_null >= len(frames_null) or end_null >= len(frames_null):
            continue

        base_null = frames_null[start_null].T
        # angle_seq_null = []
        trans_seq_null = []

        

        for i in range(start_null + 1, end_null + 1):
            current_null = frames_null[i].T
            # check_marker_order_and_translation(base_null, current_null, markers_used, i)
            # sleep(0.1)  # Wacht even om de translatie te kunnen zien in de console
            if i == end_null:
                print(f"\n Laatste frame {i}:")	
            
            t_null = euclid_distance(base_null, current_null)
            # angle_null = np.degrees(np.arccos(np.clip((np.trace(r_null) - 1) / 2, -1.0, 1.0)))
            # angle_seq_null.append(angle_null)
            trans_seq_null.append(np.linalg.norm(t_null))
            print(f"Frame {i}: Translatie = {trans_seq_null[-1]:.2f} mm")
            sleep(0.1)
        exit()

        max_rotation_dict_null[target_null].append(np.max(angle_seq_null))
        max_translation_dict_null[target_null].append(np.max(trans_seq_null))

        max_angle_null = np.max(angle_seq_null)
        max_trans_null = np.max(trans_seq_null)
        print(f"Max translatie voor {target_null}: {max_trans_null:.2f}mm")