import pandas as pd
import numpy as np
import os



namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", 
        "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]

algoritme_mapping = {"Null": 0, "Alg1": 1, "Alg2": 2}
all_results = []


for naam in namen:
    print(f"\n\n====================\nAnalyse voor {naam}:\n====================\n")
    # Null data
    file_null = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Null.csv")
    targets_null = file_null.groupby("Target")

    target_dict_null = {
        "Tar1": targets_null.get_group("Tar1").reset_index(drop=True),
        "Tar2": targets_null.get_group("Tar2").reset_index(drop=True),
        "Tar3": targets_null.get_group("Tar3").reset_index(drop=True) 
    }
    means_null_completion_time = []
    maes_null_completion_time = []
    sds_null_completion_time = []

    # Voor 10 pogingen
    for i in range(9):
        # Pak de i-de poging van elke target
        null_completion_time = [
            target_dict_null["Tar1"].loc[i, "Duur (s)"],
            target_dict_null["Tar2"].loc[i, "Duur (s)"],
            target_dict_null["Tar3"].loc[i, "Duur (s)"]
        ]
    
        # Bereken gemiddelde duur
        mean_null_completion_time = np.mean(null_completion_time)
        
        # MAE: gemiddelde absolute afwijking van het gemiddelde
        mae_null_completion_time = np.mean([abs(d - mean_null_completion_time) for d in null_completion_time])
        
        # SD: standaarddeviatie
        sd_null_completion_time = np.std(null_completion_time, ddof=1)  # ddof=1 voor sample SD
        
        means_null_completion_time.append(mean_null_completion_time)
        maes_null_completion_time.append(mae_null_completion_time)
        sds_null_completion_time.append(sd_null_completion_time)

    print("Null:")
    # Resultaten tonen
    for i, (mean_null_completion_time, mae_null_completion_time, sd_null_completion_time) in enumerate(zip(means_null_completion_time, maes_null_completion_time, sds_null_completion_time), start=1):
        print(f"Poging {i}: MEAN = {mean_null_completion_time:.4f}, MAE = {mae_null_completion_time:.4f}, SD = {sd_null_completion_time:.4f}")


    # Alg1
    file_alg1 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg1.csv")
    targets_alg1 = file_alg1.groupby("Target")

    target_dict_alg1 = {
        "Tar1": targets_alg1.get_group("Tar1").reset_index(drop=True),
        "Tar2": targets_alg1.get_group("Tar2").reset_index(drop=True),
        "Tar3": targets_alg1.get_group("Tar3").reset_index(drop=True) 
    }
    means_alg1_completion_time = []
    maes_alg1_completion_time = []
    sds_alg1_completion_time = []

    # Voor 10 pogingen
    for i in range(9):
        # Pak de i-de poging van elke target
        alg1_completion_time = [
            target_dict_alg1["Tar1"].loc[i, "Duur (s)"],
            target_dict_alg1["Tar2"].loc[i, "Duur (s)"],
            target_dict_alg1["Tar3"].loc[i, "Duur (s)"]
        ]
    
        # Bereken gemiddelde duur
        mean_alg1_completion_time = np.mean(alg1_completion_time)
        
        # MAE: gemiddelde absolute afwijking van het gemiddelde
        mae_alg1_completion_time = np.mean([abs(d - mean_alg1_completion_time) for d in alg1_completion_time])
        
        # SD: standaarddeviatie
        sd_alg1_completion_time = np.std(alg1_completion_time, ddof=1)  # ddof=1 voor sample SD
        
        means_alg1_completion_time.append(mean_alg1_completion_time)
        maes_alg1_completion_time.append(mae_alg1_completion_time)
        sds_alg1_completion_time.append(sd_alg1_completion_time)
    
    print("Algoritme 1:")
    # Resultaten tonen
    for i, (mean_alg1_completion_time, mae_alg1_completion_time, sd_alg1_completion_time) in enumerate(zip(means_alg1_completion_time, maes_alg1_completion_time, sds_alg1_completion_time), start=1):
        print(f"Poging {i}: MEAN = {mean_alg1_completion_time:.4f}, MAE = {mae_alg1_completion_time:.4f}, SD = {sd_alg1_completion_time:.4f}")


   # Alg2
    file_alg2 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg2.csv")
    targets_alg2 = file_alg2.groupby("Target")

    target_dict_alg2 = {
        "Tar1": targets_alg2.get_group("Tar1").reset_index(drop=True),
        "Tar2": targets_alg2.get_group("Tar2").reset_index(drop=True),
        "Tar3": targets_alg2.get_group("Tar3").reset_index(drop=True) 
    }
    means_alg2_completion_time = []
    maes_alg2_completion_time = []
    sds_alg2_completion_time = []

    # Voor 10 pogingen
    for i in range(9):
        # Pak de i-de poging van elke target
        alg2_completion_time = [
            target_dict_alg2["Tar1"].loc[i, "Duur (s)"],
            target_dict_alg2["Tar2"].loc[i, "Duur (s)"],
            target_dict_alg2["Tar3"].loc[i, "Duur (s)"]
        ]
    
        # Bereken gemiddelde duur
        mean_alg2_completion_time = np.mean(alg2_completion_time)
        
        # MAE: gemiddelde absolute afwijking van het gemiddelde
        mae_alg2_completion_time = np.mean([abs(d - mean_alg2_completion_time) for d in alg2_completion_time])
        
        # SD: standaarddeviatie
        sd_alg2_completion_time = np.std(alg2_completion_time, ddof=1)  # ddof=1 voor sample SD
        
        means_alg2_completion_time.append(mean_alg2_completion_time)
        maes_alg2_completion_time.append(mae_alg2_completion_time)
        sds_alg2_completion_time.append(sd_alg2_completion_time)
    
    print("Algoritme 1:")
    # Resultaten tonen
    for i, (mean_alg2_completion_time, mae_alg2_completion_time, sd_alg2_completion_time) in enumerate(zip(means_alg2_completion_time, maes_alg2_completion_time, sds_alg2_completion_time), start=1):
        print(f"Poging {i}: MEAN = {mean_alg2_completion_time:.4f}, MAE = {mae_alg2_completion_time:.4f}, SD = {sd_alg2_completion_time:.4f}")

    # Resultaten toevoegen aan de lijst (Null = 0)
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 0,
            "Mean": means_null_completion_time[i],
            "SD_Mean": sds_null_completion_time[i],
            "MAE": maes_null_completion_time[i]
        })

    # Alg1 = 1
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 1,
            "Mean": means_alg1_completion_time[i],
            "SD_Mean": sds_alg1_completion_time[i],
            "MAE": maes_alg1_completion_time[i]
        })

    # Alg2 = 2
    for i in range(9):
        all_results.append({
            "Subject": naam,
            "Trial": i + 1,
            "Algorithm": 2,
            "Mean": means_alg2_completion_time[i],
            "SD_Mean": sds_alg2_completion_time[i],
            "MAE": maes_alg2_completion_time[i]
        })

# === Exporteer naar Excel ===
df_output = pd.DataFrame(all_results)

# Optioneel: subject index i.p.v. naam gebruiken
subject_to_index = {naam: i+1 for i, naam in enumerate(namen)}
df_output["Subject"] = df_output["Subject"].map(subject_to_index)

# Pad naar bestand aanpassen indien nodig
output_path = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\completion_time_results.xlsx"
df_output.to_excel(output_path, index=False)

print(f"\n✅ Resultaten opgeslagen in Excel op:\n{output_path}")
