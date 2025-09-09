import pandas as pd
import numpy as np
import json
import os

namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", 
         "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]

# Standaard markers voor iedereen
default_markers_used = ["UlnarStyloid", "Olecranon", "Acromium"]


# Override voor specifieke proefpersonen (indien nodig)
markers_per_person = {
    # "AnnaZoet": ["Ster_cen", "Ster_R", "Ster_L"],
    # voeg hier andere uitzonderingen toe als nodig
}

def calculate_elbow_angle(acromion, olecranon, ulnar_styloid):
    vec_upper = acromion - olecranon
    vec_lower = ulnar_styloid - olecranon
    vec_upper_norm = vec_upper / np.linalg.norm(vec_upper)
    vec_lower_norm = vec_lower / np.linalg.norm(vec_lower)
    cos_angle = np.dot(vec_upper_norm, vec_lower_norm)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    return np.degrees(angle_rad)

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
    frames_null = [np.stack([marker_data_null[m][i] for m in markers_used], axis=0) for i in range(n_frames_null)]


    max_angle_null = {}

# determine maximum elbow angle over all trials
    for _, row in trial_file_null.iterrows():
        start_null = int(float(row["Startframe"]))
        end_null = int(float(row["Eindframe"]))
        
        if start_null >= len(frames_null) or end_null >= len(frames_null):
            continue
        
        angles_trial = []
        for i in range(start_null + 1, end_null + 1):
            raw_angle = calculate_elbow_angle(
                acromion=frames_null[i][markers_used.index("Acromium")],
                olecranon=frames_null[i][markers_used.index("Olecranon")],
                ulnar_styloid=frames_null[i][markers_used.index("UlnarStyloid")]
            )
            # Flip angle zodat gestrekt klein is
            flipped_angle = 180 - raw_angle
            angles_trial.append(flipped_angle)

        # Bepaal offset zodat eerste frame op 5 graden ligt
        offset = angles_trial[0] - 10
        angles_corrected = [angle - offset for angle in angles_trial]

        for idx, angle in enumerate(angles_corrected):
            frame_num = start_null + 1 + idx
            print(f"Frame {frame_num}: Ellebooghoek (gecorrigeerd) = {angle:.2f} graden")

    # for _, row in trial_file_null.iterrows():
    #     start_null = int(float(row["Startframe"]))
    #     end_null = int(float(row["Eindframe"]))

    #     # Print trial info
    #     print(f"\nTrial {int(row['Trial'])} van persoon {naam} begint op frame {start_null}:")

    #     if start_null >= len(frames_null) or end_null >= len(frames_null):
    #         print("⚠️ Trial frames buiten data range, deze trial wordt overgeslagen.")
    #         continue

    #     for i in range(start_null, end_null + 1):
    #         elbow_angle_calc = calculate_elbow_angle(
    #             acromion=frames_null[i][markers_used.index("Acromium")],
    #             olecranon=frames_null[i][markers_used.index("Olecranon")],
    #             ulnar_styloid=frames_null[i][markers_used.index("UlnarStyloid")]
    #         )
    #         elbow_angle_calc = 180 - elbow_angle_calc  # Omrekenen naar gewenste hoek

    #         print(f"Frame {i} = {elbow_angle_calc:.2f} graden")

    # for _, row in trial_file_null.iterrows():
    #     start_null = int(float(row["Startframe"]))
    #     end_null = int(float(row["Eindframe"]))


    #     if start_null >= len(frames_null) or end_null >= len(frames_null):
    #         continue
        

    #     for i in range(start_null + 1, end_null + 1):

    #         elbow_angle_calc = calculate_elbow_angle(
    #             acromion=frames_null[i][markers_used.index("Acromium")],
    #             olecranon=frames_null[i][markers_used.index("Olecranon")],
    #             ulnar_styloid=frames_null[i][markers_used.index("UlnarStyloid")]
    #         )

    #         elbow_angle_calc = 180 - elbow_angle_calc  # Omrekenen naar de gewenste hoek
    #         print(f"Frame {i}: Ellebooghoek = {elbow_angle_calc:.2f} graden")


# import pandas as pd
# import numpy as np
# import json
# import os

# namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", 
#          "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]

# # Standaard markers voor iedereen
# default_markers_used = ["UlnarStyloid", "Olecranon", "Acromium"]

# # Override voor specifieke proefpersonen (indien nodig)
# markers_per_person = {
#     # "AnnaZoet": ["Ster_cen", "Ster_R", "Ster_L"],
#     # voeg hier andere uitzonderingen toe als nodig
# }

# def calculate_elbow_angle(acromion, olecranon, ulnar_styloid):
#     vec_upper = acromion - olecranon
#     vec_lower = ulnar_styloid - olecranon
#     vec_upper_norm = vec_upper / np.linalg.norm(vec_upper)
#     vec_lower_norm = vec_lower / np.linalg.norm(vec_lower)
#     cos_angle = np.dot(vec_upper_norm, vec_lower_norm)
#     cos_angle = np.clip(cos_angle, -1.0, 1.0)
#     angle_rad = np.arccos(cos_angle)
#     return np.degrees(angle_rad)

# for naam in namen:
#     print(f"\n====================\nAnalyse voor {naam}:\n====================")
    
#     # Null data
#     marker_data_path_null = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{naam}\\{naam}_Null.json"
#     trial_file_null = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Null.csv")

#     with open(marker_data_path_null, 'r') as f:
#         marker_data_json_null = json.load(f)
#     marker_data_null = {}

#     # Kies markers voor deze persoon
#     markers_used = markers_per_person.get(naam, default_markers_used)

#     for marker in marker_data_json_null["Markers"]:
#         if marker["Name"] in markers_used:
#             marker_data_null[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]
    
#     n_frames_null = min(marker_data_null[m].shape[0] for m in markers_used)
#     frames_null = [np.stack([marker_data_null[m][i] for m in markers_used], axis=0) for i in range(n_frames_null)]

#     # Max angle per trial opslaan
#     max_angles_per_trial = {}

#     for _, row in trial_file_null.iterrows():
#         start_null = int(float(row["Startframe"]))
#         end_null = int(float(row["Eindframe"]))
#         trial_num = int(row["Trial"])
#         target = row["Target"]

#         if start_null >= len(frames_null) or end_null >= len(frames_null):
#             print(f"⚠️  {target} - Trial {trial_num} → Frames buiten bereik, overslaan")
#             continue

#         angles = []

#         for i in range(start_null + 1, end_null + 1):
#             elbow_angle_calc = calculate_elbow_angle(
#                 acromion=frames_null[i][markers_used.index("Acromium")],
#                 olecranon=frames_null[i][markers_used.index("Olecranon")],
#                 ulnar_styloid=frames_null[i][markers_used.index("UlnarStyloid")]
#             )
#             elbow_angle_calc = 180 - elbow_angle_calc  # Omrekenen naar de gewenste hoek
#             angles.append(elbow_angle_calc)
#             # print(f"Frame {i}: Ellebooghoek = {elbow_angle_calc:.2f} graden")  # Optioneel uitzetten om output te beperken

#         if angles:
#             max_angle = max(angles)
#             max_angles_per_trial[trial_num] = max_angle
#             print(f"📌 {target} - Trial {trial_num:2d} → Max ellebooghoek: {max_angle:.2f} graden")
#         else:
#             print(f"⚠️  {target} - Trial {trial_num:2d} → Geen valide data (tracking errors?)")
max_angles_per_person = {}

for naam in namen:
    print(f"\nAnalyseren van maximale ellebooghoek voor {naam}...")

    # Pad naar data
    marker_data_path_null = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{naam}\\{naam}_Null.json"
    trial_file_null = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Null.csv")

    with open(marker_data_path_null, 'r') as f:
        marker_data_json_null = json.load(f)
    marker_data_null = {}

    markers_used = markers_per_person.get(naam, default_markers_used)
    for marker in marker_data_json_null["Markers"]:
        if marker["Name"] in markers_used:
            marker_data_null[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]

    n_frames_null = min(marker_data_null[m].shape[0] for m in markers_used)
    frames_null = [np.stack([marker_data_null[m][i] for m in markers_used], axis=0) for i in range(n_frames_null)]

    max_angle_all_trials = -np.inf

    for _, row in trial_file_null.iterrows():
        start_null = int(float(row["Startframe"]))
        end_null = int(float(row["Eindframe"]))

        if start_null >= len(frames_null) or end_null >= len(frames_null):
            continue

        angles_trial = []
        for i in range(start_null + 1, end_null + 1):
            raw_angle = calculate_elbow_angle(
                acromion=frames_null[i][markers_used.index("Acromium")],
                olecranon=frames_null[i][markers_used.index("Olecranon")],
                ulnar_styloid=frames_null[i][markers_used.index("UlnarStyloid")]
            )
            flipped_angle = 180 - raw_angle
            angles_trial.append(flipped_angle)

        # Offset correctie zodat starthoek 5 graden is
        offset = angles_trial[0] - 5
        angles_corrected = [angle - offset for angle in angles_trial]

        max_angle_trial = max(angles_corrected)
        if max_angle_trial > max_angle_all_trials:
            max_angle_all_trials = max_angle_trial

    max_angles_per_person[naam] = max_angle_all_trials
    print(f"Maximale ellebooghoek voor {naam}: {max_angle_all_trials:.2f} graden")

# Wil je de resultaten later gebruiken?
# print(max_angles_per_person)
