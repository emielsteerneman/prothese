import os
import pickle
import numpy as np
import pandas as pd
import json
from collections import defaultdict
import time
from time import sleep


namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", 
        "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]

algoritme_mapping = {"Null": 0, "Alg1": 1, "Alg2": 2}
all_results = []

# Standaard markers voor iedereen
default_markers_used = ["Ster_cen", "Ster_up", "Ster_L"]

# Override voor specifieke proefpersonen (indien nodig)
markers_per_person = {
    "AnnaZoet": ["Ster_cen", "Ster_R", "Ster_L"],
    # voeg hier andere uitzonderingen toe als nodig
}

# Print de marker namen en check translatie per marker tussen base en current
def check_marker_order_and_translation(base, current, markers_used, frame_index):
    print(f"\nFrame {frame_index}:")
    for i, marker_name in enumerate(markers_used):
        base_point = base[:, i]
        current_point = current[:, i]
        dist = np.linalg.norm(current_point - base_point)
        print(f"  Marker {marker_name}: Translatie = {dist:.2f} mm")


# def r_svd(base1, base2): # OG!!!
#     base1_mean = np.mean(base1, axis=1, keepdims=True)
#     base2_mean = np.mean(base2, axis=1, keepdims=True)
#     base1_zero = base1 - base1_mean
#     base2_zero = base2 - base2_mean
#     u, _, vh = np.linalg.svd(base2_zero @ base1_zero.T)
#     r = (u * np.array([1, 1, np.linalg.det(u @ vh)])) @ vh
#     t = base2_mean - r @ base1_mean
#     return r, t, ((base2 - (r @ base1 + t)) ** 2).mean() ** 0.5

def r_svd_rot_and_euclid(base1, base2):
    base1_mean = np.mean(base1, axis=0, keepdims=True)
    base2_mean = np.mean(base2, axis=0, keepdims=True)
    # Euclidische afstand tussen centroiden
    euclid_dist = np.linalg.norm(base2_mean - base1_mean)
    base1_zero = base1 - base1_mean
    base2_zero = base2 - base2_mean
    u, _, vh = np.linalg.svd(base2_zero.T @ base1_zero)
    d = np.diag([1, 1, np.linalg.det(u @ vh)])
    r = u @ d @ vh
    # Bereken rotatiehoek in graden
    # angle_deg = np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))
    # Optioneel: RMS fout
    transformed = (base1 @ r.T) + base2_mean - (r @ base1_mean.T).T
    rmse = np.sqrt(np.mean(np.sum((base2 - transformed) ** 2, axis=1)))
    
    return r, euclid_dist, rmse


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
    # frames_null = [np.stack([marker_data_null[m][i] for m in markers_used], axis=1).T for i in range(n_frames_null)]
    frames_null = [np.stack([marker_data_null[m][i] for m in markers_used], axis=0) for i in range(n_frames_null)]

    max_rotation_dict_null = defaultdict(list)
    max_translation_dict_null = defaultdict(list)

    for _, row in trial_file_null.iterrows():
        start_null = int(float(row["Startframe"]))
        end_null = int(float(row["Eindframe"]))
        target_null = row["Target"]

        if start_null >= len(frames_null) or end_null >= len(frames_null):
            continue

        # base_null = frames_null[start_null].T
        base_null = frames_null[start_null]

        angle_seq_null = []
        trans_seq_null = []

        

        for i in range(start_null + 1, end_null + 1):
            # current_null = frames_null[i].T
            current_null = frames_null[i]

            # check_marker_order_and_translation(base_null, current_null, markers_used, i)
            # sleep(0.1)  # Wacht even om de translatie te kunnen zien in de console
            # if i == end_null:
            #     print(f"\n Laatste frame {i}:")	
            
            r_null, t_null, _ = r_svd_rot_and_euclid(base_null, current_null)
            angle_null = np.degrees(np.arccos(np.clip((np.trace(r_null) - 1) / 2, -1.0, 1.0)))
            angle_seq_null.append(angle_null)
            trans_seq_null.append(np.linalg.norm(t_null))
            # r, t, rmse = r_svd(base_null, current_null)
            # print("Rotatiematrix:\n", r)
            # print("Translatie vector:", t.flatten())
            # print("RMSE:", rmse)
            # print("base_null centroid:", base_null.mean(axis=0))
            # print("current_null centroid:", current_null.mean(axis=0))
            # print("Translatie vector t:", t_null.flatten())

            # print(f"Frame {i}: rotatie = {angle_seq_null[-1]:.2f} mm")
            # sleep(0.1)
        # exit()

        max_rotation_dict_null[target_null].append(np.max(angle_seq_null))
        max_translation_dict_null[target_null].append(np.max(trans_seq_null))

        max_angle_null = np.max(angle_seq_null)
        max_trans_null = np.max(trans_seq_null)
        print(f"Max translatie voor {target_null}: {max_trans_null:.2f}mm")

    # Analyses per poging (ronde 1 t/m 9)
    means_rotation_null, maes_rotation_null, sds_rotation_null = [], [], []
    means_translation_null, maes_translation_null, sds_translation_null = [], [], []

    for i in range(9):  # 9 pogingen per target
        try:
            rot_vals_null = [
                max_rotation_dict_null["Tar1"][i],
                max_rotation_dict_null["Tar2"][i],
                max_rotation_dict_null["Tar3"][i]
            ]
            trans_vals_null = [
                max_translation_dict_null["Tar1"][i],
                max_translation_dict_null["Tar2"][i],
                max_translation_dict_null["Tar3"][i]
            ]
        except IndexError:
            print(f"Poging {i+1}: Onvoldoende data (mogelijk ontbrekende trials).")
            continue

        # ROTATIE
        mean_r_null = np.mean(rot_vals_null)
        mae_r_null = np.mean(np.abs(np.array(rot_vals_null) - mean_r_null))
        sd_r_null = np.std(rot_vals_null, ddof=1)

        means_rotation_null.append(mean_r_null)
        maes_rotation_null.append(mae_r_null)
        sds_rotation_null.append(sd_r_null)

        # TRANSLATIE
        mean_t_null = np.mean(trans_vals_null)
        mae_t_null = np.mean(np.abs(np.array(trans_vals_null) - mean_t_null))
        sd_t_null = np.std(trans_vals_null, ddof=1)

        means_translation_null.append(mean_t_null)
        maes_translation_null.append(mae_t_null)
        sds_translation_null.append(sd_t_null)
        
    for i in range(9):
        try:
            vals = [max_translation_dict_null[t][i] for t in ["Tar1", "Tar2", "Tar3"]]
            avg_val = np.mean(vals)
            print(f"Poging {i+1}: Max translatie per target = {vals}, Gemiddelde = {avg_val:.2f}")
        except IndexError:
            print(f"Poging {i+1}: Onvoldoende data")

    # print("base (shape: {}):\n{}".format(base_null.shape, base_null))
    # print("current (shape: {}):\n{}".format(current_null.shape, current_null))



    # Resultaten printen
    print("Null:")
    print("\nRotatie resultaten:")
    for i, (mean_r_null, mae_r_null, sd_r_null) in enumerate(zip(means_rotation_null, maes_rotation_null, sds_rotation_null), start=1):
        print(f"Poging {i}: MEAN = {mean_r_null:.2f}°, MAE = {mae_r_null:.2f}°, SD = {sd_r_null:.2f}°")

    print("\nTranslatie resultaten:")
    for i, (mean_t_null, mae_t_null, sd_t_null) in enumerate(zip(means_translation_null, maes_translation_null, sds_translation_null), start=1):
        print(f"Poging {i}: MEAN = {mean_t_null:.2f} mm, MAE = {mae_t_null:.2f} mm, SD = {sd_t_null:.2f} mm")

    # Algorithm 1 data
    marker_data_path_alg1 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{naam}\\{naam}_Alg1.json"
    trial_file_alg1 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg1.csv")

    with open(marker_data_path_alg1, 'r') as f:
        marker_data_json_alg1 = json.load(f)
    marker_data_alg1 = {}

    # Kies markers voor deze persoon
    markers_used = markers_per_person.get(naam, default_markers_used)

    for marker in marker_data_json_alg1["Markers"]:
        if marker["Name"] in markers_used:
            marker_data_alg1[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]
    
    n_frames_alg1 = min(marker_data_alg1[m].shape[0] for m in markers_used)
    # frames_alg1 = [np.stack([marker_data_alg1[m][i] for m in markers_used], axis=1).T for i in range(n_frames_alg1)]
    frames_alg1 = [np.stack([marker_data_alg1[m][i] for m in markers_used], axis=0) for i in range(n_frames_alg1)]


    max_rotation_dict_alg1 = defaultdict(list)
    max_translation_dict_alg1 = defaultdict(list)

    for _, row in trial_file_alg1.iterrows():
        start_alg1 = int(float(row["Startframe"]))
        end_alg1 = int(float(row["Eindframe"]))
        target_alg1 = row["Target"]

        if start_alg1 >= len(frames_alg1) or end_alg1 >= len(frames_alg1):
            continue

        # base_alg1 = frames_alg1[start_alg1].T
        base_alg1 = frames_alg1[start_alg1]
        angle_seq_alg1 = []
        trans_seq_alg1 = []

        for i in range(start_alg1 + 1, end_alg1 + 1):
            # current_alg1 = frames_alg1[i].T
            current_alg1 = frames_alg1[i]
            r_alg1, t_alg1, _ = r_svd_rot_and_euclid(base_alg1, current_alg1)
            angle_alg1 = np.degrees(np.arccos(np.clip((np.trace(r_alg1) - 1) / 2, -1.0, 1.0)))
            angle_seq_alg1.append(angle_alg1)
            trans_seq_alg1.append(np.linalg.norm(t_alg1))

        max_rotation_dict_alg1[target_alg1].append(np.max(angle_seq_alg1))
        max_translation_dict_alg1[target_alg1].append(np.max(trans_seq_alg1))

        max_angle_alg1 = np.max(angle_seq_alg1)
        max_trans_alg1 = np.max(trans_seq_alg1)

    # Analyses per poging (ronde 1 t/m 9)
    means_rotation_alg1, maes_rotation_alg1, sds_rotation_alg1 = [], [], []
    means_translation_alg1, maes_translation_alg1, sds_translation_alg1 = [], [], []

    for i in range(9):  # 9 pogingen per target
        try:
            rot_vals_alg1 = [
                max_rotation_dict_alg1["Tar1"][i],
                max_rotation_dict_alg1["Tar2"][i],
                max_rotation_dict_alg1["Tar3"][i]
            ]
            trans_vals_alg1 = [
                max_translation_dict_alg1["Tar1"][i],
                max_translation_dict_alg1["Tar2"][i],
                max_translation_dict_alg1["Tar3"][i]
            ]
        except IndexError:
            print(f"Poging {i+1}: Onvoldoende data (mogelijk ontbrekende trials).")
            continue

        # ROTATIE
        mean_r_alg1 = np.mean(rot_vals_alg1)
        mae_r_alg1 = np.mean(np.abs(np.array(rot_vals_alg1) - mean_r_alg1))
        sd_r_alg1 = np.std(rot_vals_alg1, ddof=1)

        means_rotation_alg1.append(mean_r_alg1)
        maes_rotation_alg1.append(mae_r_alg1)
        sds_rotation_alg1.append(sd_r_alg1)

        # TRANSLATIE
        mean_t_alg1 = np.mean(trans_vals_alg1)
        mae_t_alg1 = np.mean(np.abs(np.array(trans_vals_alg1) - mean_t_alg1))
        sd_t_alg1 = np.std(trans_vals_alg1, ddof=1)

        means_translation_alg1.append(mean_t_alg1)
        maes_translation_alg1.append(mae_t_alg1)
        sds_translation_alg1.append(sd_t_alg1)

    # Resultaten printen
    print("Algoritme 1:")
    print("\nRotatie resultaten:")
    for i, (mean_r_alg1, mae_r_alg1, sd_r_alg1) in enumerate(zip(means_rotation_alg1, maes_rotation_alg1, sds_rotation_alg1), start=1):
        print(f"Poging {i}: MEAN = {mean_r_alg1:.2f}°, MAE = {mae_r_alg1:.2f}°, SD = {sd_r_alg1:.2f}°")

    print("\nTranslatie resultaten:")
    for i, (mean_t_alg1, mae_t_alg1, sd_t_alg1) in enumerate(zip(means_translation_alg1, maes_translation_alg1, sds_translation_alg1), start=1):
        print(f"Poging {i}: MEAN = {mean_t_alg1:.2f} mm, MAE = {mae_t_alg1:.2f} mm, SD = {sd_t_alg1:.2f} mm")


    # Algorithm 2 data
    marker_data_path_alg2 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{naam}\\{naam}_Alg2.json"
    trial_file_alg2 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg2.csv")

    with open(marker_data_path_alg2, 'r') as f:
        marker_data_json_alg2 = json.load(f)
    marker_data_alg2 = {}

    # Kies markers voor deze persoon
    markers_used = markers_per_person.get(naam, default_markers_used)

    for marker in marker_data_json_alg2["Markers"]:
        if marker["Name"] in markers_used:
            marker_data_alg2[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]
    
    n_frames_alg2 = min(marker_data_alg2[m].shape[0] for m in markers_used)
    # frames_alg2 = [np.stack([marker_data_alg2[m][i] for m in markers_used], axis=1).T for i in range(n_frames_alg2)]
    frames_alg2 = [np.stack([marker_data_alg2[m][i] for m in markers_used], axis=0) for i in range(n_frames_alg2)]

    

    max_rotation_dict_alg2 = defaultdict(list)
    max_translation_dict_alg2 = defaultdict(list)

    for _, row in trial_file_alg2.iterrows():
        start_alg2 = int(float(row["Startframe"]))
        end_alg2 = int(float(row["Eindframe"]))
        target_alg2 = row["Target"]

        if start_alg2 >= len(frames_alg2) or end_alg2 >= len(frames_alg2):
            continue

        # base_alg2 = frames_alg2[start_alg2].T
        base_alg2 = frames_alg2[start_alg2]
        angle_seq_alg2 = []
        trans_seq_alg2 = []

        for i in range(start_alg2 + 1, end_alg2 + 1):
            # current_alg2 = frames_alg2[i].T
            current_alg2 = frames_alg2[i]
            r_alg2, t_alg2, _ = r_svd_rot_and_euclid(base_alg2, current_alg2)
            angle_alg2 = np.degrees(np.arccos(np.clip((np.trace(r_alg2) - 1) / 2, -1.0, 1.0)))
            angle_seq_alg2.append(angle_alg2)
            trans_seq_alg2.append(np.linalg.norm(t_alg2))

        max_rotation_dict_alg2[target_alg2].append(np.max(angle_seq_alg2))
        max_translation_dict_alg2[target_alg2].append(np.max(trans_seq_alg2))

        max_angle_alg2 = np.max(angle_seq_alg2)
        max_trans_alg2 = np.max(trans_seq_alg2)

    # Analyses per poging (ronde 1 t/m 9)
    means_rotation_alg2, maes_rotation_alg2, sds_rotation_alg2 = [], [], []
    means_translation_alg2, maes_translation_alg2, sds_translation_alg2 = [], [], []

    for i in range(9):  # 9 pogingen per target
        try:
            rot_vals_alg2 = [
                max_rotation_dict_alg2["Tar1"][i],
                max_rotation_dict_alg2["Tar2"][i],
                max_rotation_dict_alg2["Tar3"][i]
            ]
            trans_vals_alg2 = [
                max_translation_dict_alg2["Tar1"][i],
                max_translation_dict_alg2["Tar2"][i],
                max_translation_dict_alg2["Tar3"][i]
            ]
        except IndexError:
            print(f"Poging {i+1}: Onvoldoende data (mogelijk ontbrekende trials).")
            continue

        # ROTATIE
        mean_r_alg2 = np.mean(rot_vals_alg2)
        mae_r_alg2 = np.mean(np.abs(np.array(rot_vals_alg2) - mean_r_alg2))
        sd_r_alg2 = np.std(rot_vals_alg2, ddof=1)

        means_rotation_alg2.append(mean_r_alg2)
        maes_rotation_alg2.append(mae_r_alg2)
        sds_rotation_alg2.append(sd_r_alg2)

        # TRANSLATIE
        mean_t_alg2 = np.mean(trans_vals_alg2)
        mae_t_alg2 = np.mean(np.abs(np.array(trans_vals_alg2) - mean_t_alg2))
        sd_t_alg2 = np.std(trans_vals_alg2, ddof=1)

        means_translation_alg2.append(mean_t_alg2)
        maes_translation_alg2.append(mae_t_alg2)
        sds_translation_alg2.append(sd_t_alg2)

    # Resultaten printen
    print("Algoritme 2:")
    print("\nRotatie resultaten:")
    for i, (mean_r_alg2, mae_r_alg2, sd_r_alg2) in enumerate(zip(means_rotation_alg2, maes_rotation_alg2, sds_rotation_alg2), start=1):
        print(f"Poging {i}: MEAN = {mean_r_alg2:.2f}°, MAE = {mae_r_alg2:.2f}°, SD = {sd_r_alg2:.2f}°")

    print("\nTranslatie resultaten:")
    for i, (mean_t_alg2, mae_t_alg2, sd_t_alg2) in enumerate(zip(means_translation_alg2, maes_translation_alg2, sds_translation_alg2), start=1):
        print(f"Poging {i}: MEAN = {mean_t_alg2:.2f} mm, MAE = {mae_t_alg2:.2f} mm, SD = {sd_t_alg2:.2f} mm")

    # all_results = []  # resetten of aanvullen naast completion_time

    # Rotatie: Null = 0
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 0,
            "Type": "Rotation",
            "Mean": means_rotation_null[i],
            "SD_Mean": sds_rotation_null[i],
            "MAE": maes_rotation_null[i]
        })
    # Herhaal hetzelfde voor Alg1 = 1
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 1,
            "Type": "Rotation",
            "Mean": means_rotation_alg1[i],
            "SD_Mean": sds_rotation_alg1[i],
            "MAE": maes_rotation_alg1[i]
        })
    # En Alg2 = 2
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 2,
            "Type": "Rotation",
            "Mean": means_rotation_alg2[i],
            "SD_Mean": sds_rotation_alg2[i],
            "MAE": maes_rotation_alg2[i]
        })

    
    # Translatie
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 0,
            "Type": "Translation",
            "Mean": means_translation_null[i],
            "SD_Mean": sds_translation_null[i],
            "MAE": maes_translation_null[i]
        })
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 1,
            "Type": "Translation",
            "Mean": means_translation_alg1[i],
            "SD_Mean": sds_translation_alg1[i],
            "MAE": maes_translation_alg1[i]
        })
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 2,
            "Type": "Translation",
            "Mean": means_translation_alg2[i],
            "SD_Mean": sds_translation_alg2[i],
            "MAE": maes_translation_alg2[i]
        })

# === Exporteer naar Excel ===
df_output = pd.DataFrame(all_results)

# Optioneel: subject index i.p.v. naam gebruiken
subject_to_index = {naam: i+1 for i, naam in enumerate(namen)}
df_output["Subject"] = df_output["Subject"].map(subject_to_index)

output_path = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\rot_euclid_results_trunk.xlsx"
df_output.to_excel(output_path, index=False)

print(f"\n✅ Resultaten opgeslagen in Excel op:\n{output_path}")








# import os
# import pickle
# import numpy as np
# import pandas as pd
# import json
# from collections import defaultdict


# namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", 
#         "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]

# algoritme_mapping = {"Null": 0, "Alg1": 1, "Alg2": 2}
# all_results = []

# # Standaard markers voor iedereen
# default_markers_used = ["Ster_cen", "Ster_up", "Ster_L"]

# # Override voor specifieke proefpersonen (indien nodig)
# markers_per_person = {
#     "AnnaZoet": ["Ster_cen", "Ster_R", "Ster_L"],
#     # voeg hier andere uitzonderingen toe als nodig
# }

# def r_svd(base1, base2):
#     base1_mean = np.mean(base1, axis=1, keepdims=True)
#     base2_mean = np.mean(base2, axis=1, keepdims=True)
#     base1_zero = base1 - base1_mean
#     base2_zero = base2 - base2_mean
#     u, _, vh = np.linalg.svd(base2_zero @ base1_zero.T)
#     r = (u * np.array([1, 1, np.linalg.det(u @ vh)])) @ vh
#     t = base2_mean - r @ base1_mean
#     return r, t, ((base2 - (r @ base1 + t)) ** 2).mean() ** 0.5


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
#     frames_null = [np.stack([marker_data_null[m][i] for m in markers_used], axis=1).T for i in range(n_frames_null)]

#     max_rotation_dict_null = defaultdict(list)
#     max_translation_dict_null = defaultdict(list)

#     for _, row in trial_file_null.iterrows():
#         start_null = int(float(row["Startframe"]))
#         end_null = int(float(row["Eindframe"]))
#         target_null = row["Target"]

#         if start_null >= len(frames_null) or end_null >= len(frames_null):
#             continue

#         base_null = frames_null[start_null].T
#         angle_seq_null = []
#         trans_seq_null = []

#         for i in range(start_null + 1, end_null + 1):
#             current_null = frames_null[i].T
#             r_null, t_null, _ = r_svd(base_null, current_null)
#             angle_null = np.degrees(np.arccos(np.clip((np.trace(r_null) - 1) / 2, -1.0, 1.0)))
#             angle_seq_null.append(angle_null)
#             trans_seq_null.append(np.linalg.norm(t_null))

#         max_rotation_dict_null[target_null].append(np.max(angle_seq_null))
#         max_translation_dict_null[target_null].append(np.max(trans_seq_null))

#         max_angle_null = np.max(angle_seq_null)
#         max_trans_null = np.max(trans_seq_null)

#     # Analyses per poging (ronde 1 t/m 9)
#     means_rotation_null, maes_rotation_null, sds_rotation_null = [], [], []
#     means_translation_null, maes_translation_null, sds_translation_null = [], [], []

#     for i in range(9):  # 9 pogingen per target
#         try:
#             rot_vals_null = [
#                 max_rotation_dict_null["Tar1"][i],
#                 max_rotation_dict_null["Tar2"][i],
#                 max_rotation_dict_null["Tar3"][i]
#             ]
#             trans_vals_null = [
#                 max_translation_dict_null["Tar1"][i],
#                 max_translation_dict_null["Tar2"][i],
#                 max_translation_dict_null["Tar3"][i]
#             ]
#         except IndexError:
#             print(f"Poging {i+1}: Onvoldoende data (mogelijk ontbrekende trials).")
#             continue

#         # ROTATIE
#         mean_r_null = np.mean(rot_vals_null)
#         mae_r_null = np.mean(np.abs(np.array(rot_vals_null) - mean_r_null))
#         sd_r_null = np.std(rot_vals_null, ddof=1)

#         means_rotation_null.append(mean_r_null)
#         maes_rotation_null.append(mae_r_null)
#         sds_rotation_null.append(sd_r_null)

#         # TRANSLATIE
#         mean_t_null = np.mean(trans_vals_null)
#         mae_t_null = np.mean(np.abs(np.array(trans_vals_null) - mean_t_null))
#         sd_t_null = np.std(trans_vals_null, ddof=1)

#         means_translation_null.append(mean_t_null)
#         maes_translation_null.append(mae_t_null)
#         sds_translation_null.append(sd_t_null)
    
#     # Resultaten printen
#     print("Null:")
#     print("\nRotatie resultaten:")
#     for i, (mean_r_null, mae_r_null, sd_r_null) in enumerate(zip(means_rotation_null, maes_rotation_null, sds_rotation_null), start=1):
#         print(f"Poging {i}: MEAN = {mean_r_null:.2f}°, MAE = {mae_r_null:.2f}°, SD = {sd_r_null:.2f}°")

#     print("\nTranslatie resultaten:")
#     for i, (mean_t_null, mae_t_null, sd_t_null) in enumerate(zip(means_translation_null, maes_translation_null, sds_translation_null), start=1):
#         print(f"Poging {i}: MEAN = {mean_t_null:.2f} mm, MAE = {mae_t_null:.2f} mm, SD = {sd_t_null:.2f} mm")

#     # Algorithm 1 data
#     marker_data_path_alg1 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{naam}\\{naam}_Alg1.json"
#     trial_file_alg1 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg1.csv")

#     with open(marker_data_path_alg1, 'r') as f:
#         marker_data_json_alg1 = json.load(f)
#     marker_data_alg1 = {}

#     # Kies markers voor deze persoon
#     markers_used = markers_per_person.get(naam, default_markers_used)

#     for marker in marker_data_json_alg1["Markers"]:
#         if marker["Name"] in markers_used:
#             marker_data_alg1[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]
    
#     n_frames_alg1 = min(marker_data_alg1[m].shape[0] for m in markers_used)
#     frames_alg1 = [np.stack([marker_data_alg1[m][i] for m in markers_used], axis=1).T for i in range(n_frames_alg1)]

#     max_rotation_dict_alg1 = defaultdict(list)
#     max_translation_dict_alg1 = defaultdict(list)

#     for _, row in trial_file_alg1.iterrows():
#         start_alg1 = int(float(row["Startframe"]))
#         end_alg1 = int(float(row["Eindframe"]))
#         target_alg1 = row["Target"]

#         if start_alg1 >= len(frames_alg1) or end_alg1 >= len(frames_alg1):
#             continue

#         base_alg1 = frames_alg1[start_alg1].T
#         angle_seq_alg1 = []
#         trans_seq_alg1 = []

#         for i in range(start_alg1 + 1, end_alg1 + 1):
#             current_alg1 = frames_alg1[i].T
#             r_alg1, t_alg1, _ = r_svd(base_alg1, current_alg1)
#             angle_alg1 = np.degrees(np.arccos(np.clip((np.trace(r_alg1) - 1) / 2, -1.0, 1.0)))
#             angle_seq_alg1.append(angle_alg1)
#             trans_seq_alg1.append(np.linalg.norm(t_alg1))

#         max_rotation_dict_alg1[target_alg1].append(np.max(angle_seq_alg1))
#         max_translation_dict_alg1[target_alg1].append(np.max(trans_seq_alg1))

#         max_angle_alg1 = np.max(angle_seq_alg1)
#         max_trans_alg1 = np.max(trans_seq_alg1)

#     # Analyses per poging (ronde 1 t/m 9)
#     means_rotation_alg1, maes_rotation_alg1, sds_rotation_alg1 = [], [], []
#     means_translation_alg1, maes_translation_alg1, sds_translation_alg1 = [], [], []

#     for i in range(9):  # 9 pogingen per target
#         try:
#             rot_vals_alg1 = [
#                 max_rotation_dict_alg1["Tar1"][i],
#                 max_rotation_dict_alg1["Tar2"][i],
#                 max_rotation_dict_alg1["Tar3"][i]
#             ]
#             trans_vals_alg1 = [
#                 max_translation_dict_alg1["Tar1"][i],
#                 max_translation_dict_alg1["Tar2"][i],
#                 max_translation_dict_alg1["Tar3"][i]
#             ]
#         except IndexError:
#             print(f"Poging {i+1}: Onvoldoende data (mogelijk ontbrekende trials).")
#             continue

#         # ROTATIE
#         mean_r_alg1 = np.mean(rot_vals_alg1)
#         mae_r_alg1 = np.mean(np.abs(np.array(rot_vals_alg1) - mean_r_alg1))
#         sd_r_alg1 = np.std(rot_vals_alg1, ddof=1)

#         means_rotation_alg1.append(mean_r_alg1)
#         maes_rotation_alg1.append(mae_r_alg1)
#         sds_rotation_alg1.append(sd_r_alg1)

#         # TRANSLATIE
#         mean_t_alg1 = np.mean(trans_vals_alg1)
#         mae_t_alg1 = np.mean(np.abs(np.array(trans_vals_alg1) - mean_t_alg1))
#         sd_t_alg1 = np.std(trans_vals_alg1, ddof=1)

#         means_translation_alg1.append(mean_t_alg1)
#         maes_translation_alg1.append(mae_t_alg1)
#         sds_translation_alg1.append(sd_t_alg1)

#     # Resultaten printen
#     print("Algoritme 1:")
#     print("\nRotatie resultaten:")
#     for i, (mean_r_alg1, mae_r_alg1, sd_r_alg1) in enumerate(zip(means_rotation_alg1, maes_rotation_alg1, sds_rotation_alg1), start=1):
#         print(f"Poging {i}: MEAN = {mean_r_alg1:.2f}°, MAE = {mae_r_alg1:.2f}°, SD = {sd_r_alg1:.2f}°")

#     print("\nTranslatie resultaten:")
#     for i, (mean_t_alg1, mae_t_alg1, sd_t_alg1) in enumerate(zip(means_translation_alg1, maes_translation_alg1, sds_translation_alg1), start=1):
#         print(f"Poging {i}: MEAN = {mean_t_alg1:.2f} mm, MAE = {mae_t_alg1:.2f} mm, SD = {sd_t_alg1:.2f} mm")


#     # Algorithm 2 data
#     marker_data_path_alg2 = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{naam}\\{naam}_Alg2.json"
#     trial_file_alg2 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg2.csv")

#     with open(marker_data_path_alg2, 'r') as f:
#         marker_data_json_alg2 = json.load(f)
#     marker_data_alg2 = {}

#     # Kies markers voor deze persoon
#     markers_used = markers_per_person.get(naam, default_markers_used)

#     for marker in marker_data_json_alg2["Markers"]:
#         if marker["Name"] in markers_used:
#             marker_data_alg2[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]
    
#     n_frames_alg2 = min(marker_data_alg2[m].shape[0] for m in markers_used)
#     frames_alg2 = [np.stack([marker_data_alg2[m][i] for m in markers_used], axis=1).T for i in range(n_frames_alg2)]

#     max_rotation_dict_alg2 = defaultdict(list)
#     max_translation_dict_alg2 = defaultdict(list)

#     for _, row in trial_file_alg2.iterrows():
#         start_alg2 = int(float(row["Startframe"]))
#         end_alg2 = int(float(row["Eindframe"]))
#         target_alg2 = row["Target"]

#         if start_alg2 >= len(frames_alg2) or end_alg2 >= len(frames_alg2):
#             continue

#         base_alg2 = frames_alg2[start_alg2].T
#         angle_seq_alg2 = []
#         trans_seq_alg2 = []

#         for i in range(start_alg2 + 1, end_alg2 + 1):
#             current_alg2 = frames_alg2[i].T
#             r_alg2, t_alg2, _ = r_svd(base_alg2, current_alg2)
#             angle_alg2 = np.degrees(np.arccos(np.clip((np.trace(r_alg2) - 1) / 2, -1.0, 1.0)))
#             angle_seq_alg2.append(angle_alg2)
#             trans_seq_alg2.append(np.linalg.norm(t_alg2))

#         max_rotation_dict_alg2[target_alg2].append(np.max(angle_seq_alg2))
#         max_translation_dict_alg2[target_alg2].append(np.max(trans_seq_alg2))

#         max_angle_alg2 = np.max(angle_seq_alg2)
#         max_trans_alg2 = np.max(trans_seq_alg2)

#     # Analyses per poging (ronde 1 t/m 9)
#     means_rotation_alg2, maes_rotation_alg2, sds_rotation_alg2 = [], [], []
#     means_translation_alg2, maes_translation_alg2, sds_translation_alg2 = [], [], []

#     for i in range(9):  # 9 pogingen per target
#         try:
#             rot_vals_alg2 = [
#                 max_rotation_dict_alg2["Tar1"][i],
#                 max_rotation_dict_alg2["Tar2"][i],
#                 max_rotation_dict_alg2["Tar3"][i]
#             ]
#             trans_vals_alg2 = [
#                 max_translation_dict_alg2["Tar1"][i],
#                 max_translation_dict_alg2["Tar2"][i],
#                 max_translation_dict_alg2["Tar3"][i]
#             ]
#         except IndexError:
#             print(f"Poging {i+1}: Onvoldoende data (mogelijk ontbrekende trials).")
#             continue

#         # ROTATIE
#         mean_r_alg2 = np.mean(rot_vals_alg2)
#         mae_r_alg2 = np.mean(np.abs(np.array(rot_vals_alg2) - mean_r_alg2))
#         sd_r_alg2 = np.std(rot_vals_alg2, ddof=1)

#         means_rotation_alg2.append(mean_r_alg2)
#         maes_rotation_alg2.append(mae_r_alg2)
#         sds_rotation_alg2.append(sd_r_alg2)

#         # TRANSLATIE
#         mean_t_alg2 = np.mean(trans_vals_alg2)
#         mae_t_alg2 = np.mean(np.abs(np.array(trans_vals_alg2) - mean_t_alg2))
#         sd_t_alg2 = np.std(trans_vals_alg2, ddof=1)

#         means_translation_alg2.append(mean_t_alg2)
#         maes_translation_alg2.append(mae_t_alg2)
#         sds_translation_alg2.append(sd_t_alg2)

#     # Resultaten printen
#     print("Algoritme 2:")
#     print("\nRotatie resultaten:")
#     for i, (mean_r_alg2, mae_r_alg2, sd_r_alg2) in enumerate(zip(means_rotation_alg2, maes_rotation_alg2, sds_rotation_alg2), start=1):
#         print(f"Poging {i}: MEAN = {mean_r_alg2:.2f}°, MAE = {mae_r_alg2:.2f}°, SD = {sd_r_alg2:.2f}°")

#     print("\nTranslatie resultaten:")
#     for i, (mean_t_alg2, mae_t_alg2, sd_t_alg2) in enumerate(zip(means_translation_alg2, maes_translation_alg2, sds_translation_alg2), start=1):
#         print(f"Poging {i}: MEAN = {mean_t_alg2:.2f} mm, MAE = {mae_t_alg2:.2f} mm, SD = {sd_t_alg2:.2f} mm")

#     # all_results = []  # resetten of aanvullen naast completion_time

#     # Rotatie: Null = 0
#     for i in range(9):
#         all_results.append({
#             "Subject": naam,
#             "Trial": i + 1,
#             "Algorithm": 0,
#             "Type": "Rotation",
#             "Mean": means_rotation_null[i],
#             "SD_Mean": sds_rotation_null[i],
#             "MAE": maes_rotation_null[i]
#         })
#     # Herhaal hetzelfde voor Alg1 = 1
#     for i in range(9):
#         all_results.append({
#             "Subject": naam,
#             "Trial": i + 1,
#             "Algorithm": 1,
#             "Type": "Rotation",
#             "Mean": means_rotation_alg1[i],
#             "SD_Mean": sds_rotation_alg1[i],
#             "MAE": maes_rotation_alg1[i]
#         })
#     # En Alg2 = 2
#     for i in range(9):
#         all_results.append({
#             "Subject": naam,
#             "Trial": i + 1,
#             "Algorithm": 2,
#             "Type": "Rotation",
#             "Mean": means_rotation_alg2[i],
#             "SD_Mean": sds_rotation_alg2[i],
#             "MAE": maes_rotation_alg2[i]
#         })

    
#     # Translatie
#     for i in range(9):
#         all_results.append({
#             "Subject": naam,
#             "Trial": i + 1,
#             "Algorithm": 0,
#             "Type": "Translation",
#             "Mean": means_translation_null[i],
#             "SD_Mean": sds_translation_null[i],
#             "MAE": maes_translation_null[i]
#         })
#     for i in range(9):
#         all_results.append({
#             "Subject": naam,
#             "Trial": i + 1,
#             "Algorithm": 1,
#             "Type": "Translation",
#             "Mean": means_translation_alg1[i],
#             "SD_Mean": sds_translation_alg1[i],
#             "MAE": maes_translation_alg1[i]
#         })
#     for i in range(9):
#         all_results.append({
#             "Subject": naam,
#             "Trial": i + 1,
#             "Algorithm": 2,
#             "Type": "Translation",
#             "Mean": means_translation_alg2[i],
#             "SD_Mean": sds_translation_alg2[i],
#             "MAE": maes_translation_alg2[i]
#         })

# # === Exporteer naar Excel ===
# df_output = pd.DataFrame(all_results)

# # Optioneel: subject index i.p.v. naam gebruiken
# subject_to_index = {naam: i+1 for i, naam in enumerate(namen)}
# df_output["Subject"] = df_output["Subject"].map(subject_to_index)

# output_path = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\rot_trans_results_trunk.xlsx"
# df_output.to_excel(output_path, index=False)

# print(f"\n✅ Resultaten opgeslagen in Excel op:\n{output_path}")





