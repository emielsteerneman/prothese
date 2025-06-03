### === CODE IS KLAAR === ###

# inladen van Null trials data
# aantal trials per target bepalen
# duur van de trial bij elkaar optellen
# gemiddelde duur berekenen
# gemiddelde duur per target berekenen
# inladen van de algoritme data
# duur van de trials per target bepalen
# Al deze waarden delen door de bijbehorende gemiddelde waarde van de null trials

import pandas as pd
import matplotlib.pyplot as plt

# name = "AnnaZoet"

# qualisys_file_Null =  pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_Null.csv")

# average_durations_Null = qualisys_file_Null.groupby('Target')['Duur (s)'].mean()

# print(average_durations_Null)

# # for target, avg in average_durations.items():
# #     print(f"Target {target}: {avg:.2f}")

# qualisys_file_Alg1 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_Alg1.csv")

# # Normaliseren van de Alg1 durations per target op basis van gemiddelde Null durations
# qualisys_file_Alg1['Genormaliseerde Duur'] = qualisys_file_Alg1.apply(
#     lambda row: row['Duur (s)'] / average_durations_Null[row['Target']], axis=1
# )
# # Voeg de gemiddelde Null duur per target toe aan elke rij
# qualisys_file_Alg1['Gemiddelde Null Duur'] = qualisys_file_Alg1['Target'].map(average_durations_Null)

# # Print relevante kolommen
# print(qualisys_file_Alg1[['Target', 'Duur (s)', 'Gemiddelde Null Duur', 'Genormaliseerde Duur']])


# import matplotlib.pyplot as plt

# # Targets en kleuren toewijzen
# targets = sorted(qualisys_file_Alg1['Target'].unique())
# colors = ['c', 'green', 'blue']

# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5), sharey=True)

# for i, target in enumerate(targets):
#     subset = qualisys_file_Alg1[qualisys_file_Alg1['Target'] == target]
#     axes[i].bar(range(1, len(subset) + 1), subset['Genormaliseerde Duur'], color=colors[i])
#     axes[i].set_title(f'Target {target}')
#     axes[i].set_xlabel('Trial nummer')
#     if i == 0:
#         axes[i].set_ylabel('Genormaliseerde Duur')

# plt.tight_layout()
# plt.show()


# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.patches as mpatches

# # Waarden uit dataframe halen
# durations = qualisys_file_Alg1['Genormaliseerde Duur'].tolist()
# trial_targets = qualisys_file_Alg1['Target'].astype(str).tolist()  # converteer naar string

# # Kleurmap met string keys
# colors = {'Tar1': 'c', 'Tar2': 'green', 'Tar3': 'blue'}
# trial_colors = [colors[target] for target in trial_targets]

# # Plot
# plt.figure(figsize=(10, 6))
# plt.bar(np.arange(1, len(durations) + 1), durations, color=trial_colors, edgecolor='black')

# plt.title('Genormaliseerde duur van alle trials per target', fontsize=16)
# plt.xlabel('Trial nummer')
# plt.ylabel('Genormaliseerde duur')
# plt.xticks(np.arange(1, len(durations) + 1))
# plt.grid(True, axis='y')

# # Legenda
# handles = [mpatches.Patch(color=color, label=target) for target, color in colors.items()]
# plt.legend(handles=handles, title='Target')

# plt.tight_layout()
# plt.show()


# # === Verwerking voor Alg2 ===

# # Inladen van Alg2-bestand
# qualisys_file_Alg2 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_Alg2.csv")

# # Normaliseren van de Alg2 durations per target op basis van gemiddelde Null durations
# qualisys_file_Alg2['Genormaliseerde Duur'] = qualisys_file_Alg2.apply(
#     lambda row: row['Duur (s)'] / average_durations_Null[row['Target']], axis=1
# )
# qualisys_file_Alg2['Gemiddelde Null Duur'] = qualisys_file_Alg2['Target'].map(average_durations_Null)

# # Print overzicht Alg2
# print(qualisys_file_Alg2[['Target', 'Duur (s)', 'Gemiddelde Null Duur', 'Genormaliseerde Duur']])

# # === Subplot per target voor Alg2 ===
# targets_Alg2 = sorted(qualisys_file_Alg2['Target'].unique())
# colors_Alg2 = ['magenta', 'orange', 'gray']

# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5), sharey=True)

# for i, target in enumerate(targets_Alg2):
#     subset = qualisys_file_Alg2[qualisys_file_Alg2['Target'] == target]
#     axes[i].bar(range(1, len(subset) + 1), subset['Genormaliseerde Duur'], color=colors_Alg2[i])
#     axes[i].set_title(f'Target {target}')
#     axes[i].set_xlabel('Trial nummer')
#     if i == 0:
#         axes[i].set_ylabel('Genormaliseerde Duur')

# plt.tight_layout()
# plt.show()

# # === Staafdiagram voor alle Alg2-trials in originele volgorde, met kleur per target ===
# import numpy as np
# import matplotlib.patches as mpatches

# durations_Alg2 = qualisys_file_Alg2['Genormaliseerde Duur'].tolist()
# trial_targets_Alg2 = qualisys_file_Alg2['Target'].astype(str).tolist()

# color_map_Alg2 = {'Tar1': 'magenta', 'Tar2': 'orange', 'Tar3': 'gray'}
# trial_colors_Alg2 = [color_map_Alg2[target] for target in trial_targets_Alg2]

# plt.figure(figsize=(10, 6))
# plt.bar(np.arange(1, len(durations_Alg2) + 1), durations_Alg2, color=trial_colors_Alg2, edgecolor='black')

# plt.title('Genormaliseerde duur van alle trials per target (Alg2)', fontsize=16)
# plt.xlabel('Trial nummer')
# plt.ylabel('Genormaliseerde duur')
# plt.xticks(np.arange(1, len(durations_Alg2) + 1))
# plt.grid(True, axis='y')

# handles_Alg2 = [mpatches.Patch(color=color, label=target) for target, color in color_map_Alg2.items()]
# plt.legend(handles=handles_Alg2, title='Target')

# plt.tight_layout()
# plt.show()



# namen = ["AnnaZoet", "LiekeZwier", "MartZoet"]
# combined_Alg1 = []

# for naam in namen:
#     # Null data
#     file_null = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Null.csv")
#     avg_null = file_null.groupby('Target')['Duur (s)'].mean()

#     # Alg1 data
#     file_alg1 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg1.csv")
#     file_alg1['Genormaliseerde Duur'] = file_alg1.apply(lambda row: row['Duur (s)'] / avg_null[row['Target']], axis=1)
#     file_alg1['Gemiddelde Null Duur'] = file_alg1['Target'].map(avg_null)
#     file_alg1['Persoon'] = naam

#     combined_Alg1.append(file_alg1)

# # Combineer alle data
# df_combined = pd.concat(combined_Alg1, ignore_index=True)

# # Sorteer eerst
# df_combined_sorted = df_combined.sort_values(by=['Persoon', 'Target']).copy()

# # Beperk tot max 10 trials per persoon per target
# df_combined_trimmed = df_combined_sorted.groupby(['Persoon', 'Target']).head(10).copy()

# # TrialIndex opnieuw toewijzen na trimmen
# df_combined_trimmed['TrialIndex'] = df_combined_trimmed.groupby(['Persoon', 'Target']).cumcount()

# gemiddeldes = df_combined_trimmed.groupby(['Target', 'TrialIndex'])['Genormaliseerde Duur'].mean().reset_index()






# === Alg2-verwerking ===

### HIERONDER GOED WERKENDE CODE ###

# namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]
# combined_Alg1 = []
# combined_Alg2 = []

# for naam in namen:
#     # Null data
#     file_null = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Null.csv")
#     avg_null = file_null.groupby('Target')['Duur (s)'].mean()

#     # Alg1
#     file_alg1 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg1.csv")
#     file_alg1['Genormaliseerde Duur'] = file_alg1.apply(lambda row: row['Duur (s)'] / avg_null[row['Target']], axis=1)
#     file_alg1['Gemiddelde Null Duur'] = file_alg1['Target'].map(avg_null)
#     file_alg1['Persoon'] = naam
#     combined_Alg1.append(file_alg1)

#     # Alg2
#     file_alg2 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg2.csv")
#     file_alg2['Genormaliseerde Duur'] = file_alg2.apply(lambda row: row['Duur (s)'] / avg_null[row['Target']], axis=1)
#     file_alg2['Gemiddelde Null Duur'] = file_alg2['Target'].map(avg_null)
#     file_alg2['Persoon'] = naam
#     combined_Alg2.append(file_alg2)

# # === Alg1 verwerken ===
# df_alg1 = pd.concat(combined_Alg1, ignore_index=True)
# df_alg1 = df_alg1.sort_values(by=['Persoon', 'Target']).groupby(['Persoon', 'Target']).head(10).copy()
# df_alg1['TrialIndex'] = df_alg1.groupby(['Persoon', 'Target']).cumcount()
# gemiddeldes_alg1 = df_alg1.groupby(['Target', 'TrialIndex'])['Genormaliseerde Duur'].mean().reset_index()

# # === Alg2 verwerken ===
# df_alg2 = pd.concat(combined_Alg2, ignore_index=True)
# df_alg2 = df_alg2.sort_values(by=['Persoon', 'Target']).groupby(['Persoon', 'Target']).head(10).copy()
# df_alg2['TrialIndex'] = df_alg2.groupby(['Persoon', 'Target']).cumcount()
# gemiddeldes_alg2 = df_alg2.groupby(['Target', 'TrialIndex'])['Genormaliseerde Duur'].mean().reset_index()

# # Plot Alg1
# targets = sorted(gemiddeldes_alg1['Target'].unique())
# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5), sharey=True)

# for i, target in enumerate(targets):
#     subset = gemiddeldes_alg1[gemiddeldes_alg1['Target'] == target]
#     axes[i].bar(subset['TrialIndex'] + 1, subset['Genormaliseerde Duur'], color='darkorange')
#     axes[i].set_title(f'Target {target}')
#     axes[i].set_xlabel('Trial number')
#     if i == 0:
#         axes[i].set_ylabel('Gemiddelde Genormaliseerde Duur')

# fig.suptitle('Gemiddelde Genormaliseerde Duur – Algoritme 1', fontsize=16)
# plt.tight_layout(rect=[0, 0, 1, 0.95])  # ruimte voor suptitle
# plt.show()


# # Plot alg 2
# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5), sharey=True)
# targets = sorted(gemiddeldes_alg2['Target'].unique())

# for i, target in enumerate(targets):
#     subset = gemiddeldes_alg2[gemiddeldes_alg2['Target'] == target]
#     axes[i].bar(subset['TrialIndex'] + 1, subset['Genormaliseerde Duur'], color='seagreen')
#     axes[i].set_title(f'Target {target}')
#     axes[i].set_xlabel('Trial nummer')
#     if i == 0:
#         axes[i].set_ylabel('Gemiddelde Genormaliseerde Duur')

# fig.suptitle('Gemiddelde Genormaliseerde Duur – Algoritme 2', fontsize=16)
# plt.tight_layout(rect=[0, 0, 1, 0.95])  # ruimte voor suptitle
# plt.show()

# print("nu komt versie 2")

import pandas as pd
import matplotlib.pyplot as plt

namen = ["AnnaZoet", "ChrisKrommendijk", "LiekeZwier", "MartZoet", "ThijsBink", 
         "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]

mapping = {'Tar1': 1, 'Tar2': 2, 'Tar3': 3}

combined_Alg1 = []
combined_Alg2 = []

for naam in namen:
    # Null data
    file_null = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Null.csv")
    avg_null = file_null.groupby('Target')['Duur (s)'].mean()

    # Alg1
    file_alg1 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg1.csv")
    file_alg1['Genormaliseerde Duur'] = file_alg1.apply(lambda row: row['Duur (s)'] / avg_null[row['Target']], axis=1)
    file_alg1['Gemiddelde Null Duur'] = file_alg1['Target'].map(avg_null)
    file_alg1['Persoon'] = naam
    combined_Alg1.append(file_alg1)

    # Alg2
    file_alg2 = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{naam}\\{naam}_trials_qualisys_Alg2.csv")
    file_alg2['Genormaliseerde Duur'] = file_alg2.apply(lambda row: row['Duur (s)'] / avg_null[row['Target']], axis=1)
    file_alg2['Gemiddelde Null Duur'] = file_alg2['Target'].map(avg_null)
    file_alg2['Persoon'] = naam
    combined_Alg2.append(file_alg2)

# === Alg1 verwerken ===
df_alg1 = pd.concat(combined_Alg1, ignore_index=True)
df_alg1 = df_alg1.sort_values(by=['Persoon', 'Target']).groupby(['Persoon', 'Target']).head(10).copy()
df_alg1['TrialIndex'] = df_alg1.groupby(['Persoon', 'Target']).cumcount()

# Bereken gemiddelde en standaarddeviatie per Target en TrialIndex
stats_alg1 = df_alg1.groupby(['Target', 'TrialIndex'])['Genormaliseerde Duur'].agg(['mean', 'std']).reset_index()
stats_alg1['Target'] = stats_alg1['Target'].map(mapping)

# === Alg2 verwerken ===
df_alg2 = pd.concat(combined_Alg2, ignore_index=True)
df_alg2 = df_alg2.sort_values(by=['Persoon', 'Target']).groupby(['Persoon', 'Target']).head(10).copy()
df_alg2['TrialIndex'] = df_alg2.groupby(['Persoon', 'Target']).cumcount()

# Bereken gemiddelde en standaarddeviatie per Target en TrialIndex
stats_alg2 = df_alg2.groupby(['Target', 'TrialIndex'])['Genormaliseerde Duur'].agg(['mean', 'std']).reset_index()
stats_alg2['Target'] = stats_alg2['Target'].map(mapping)

# Plot Alg1 met standaarddeviatie
# targets = sorted(stats_alg1['Target'].unique())
# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5), sharey=True)

# for i, target in enumerate(targets):
#     subset = stats_alg1[stats_alg1['Target'] == target]
#     axes[i].bar(subset['TrialIndex'] + 1, subset['mean'], yerr=subset['std'], capsize=5, color='cornflowerblue')
#     axes[i].set_title(f'Target {target}')
#     axes[i].set_xlabel('Trial Number')
#     if i == 0:
#         axes[i].set_ylabel('Mean Normalized Completion Time')

# fig.suptitle('Mean Normalized Completion Time ± SD – Algorithm 1', fontsize=16)
# plt.tight_layout(rect=[0, 0, 1, 0.95])
# plt.show()

# # Plot Alg2 met standaarddeviatie
# targets = sorted(stats_alg2['Target'].unique())
# fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(15, 5), sharey=True)

# for i, target in enumerate(targets):
#     subset = stats_alg2[stats_alg2['Target'] == target]
#     axes[i].bar(subset['TrialIndex'] + 1, subset['mean'], yerr=subset['std'], capsize=5, color='seagreen')
#     axes[i].set_title(f'Target {target}')
#     axes[i].set_xlabel('Trial Number')
#     if i == 0:
#         axes[i].set_ylabel('Mean Normalized Completion Time')

# fig.suptitle('Mean Normalized Completion Time ± SD – Algorithm 2', fontsize=16)
# plt.tight_layout(rect=[0, 0, 1, 0.95])
# plt.show()


# for target in sorted(stats_alg1['Target'].unique()):
#     subset = stats_alg1[stats_alg1['Target'] == target]
#     plt.figure(figsize=(16,4))
#     plt.bar(subset['TrialIndex'] + 1, subset['mean'], yerr=subset['std'], capsize=5, color='cornflowerblue')
#     plt.title(f'Mean Normalized Completion Time ± SD - Algorithm 1 - Target {target}', fontsize=20)  # Grotere titel
#     plt.xlabel('Trial Number', fontsize=18)  # Grotere x-as label
#     plt.ylabel('Completion Time', fontsize=18)  # Grotere y-as label
#     plt.xticks(range(1, subset['TrialIndex'].max() + 2), fontsize = 16)  # Alle trial nummers tonen
#     plt.yticks(range(0,14,2), fontsize=16)
#     plt.tight_layout()
#     plt.show()

# for target in sorted(stats_alg1['Target'].unique()):
#     subset = stats_alg2[stats_alg2['Target'] == target]
#     plt.figure(figsize=(16,4))
#     plt.bar(subset['TrialIndex'] + 1, subset['mean'], yerr=subset['std'], capsize=5, color='mediumseagreen')
#     plt.title(f'Mean Normalized Completion Time ± SD - Algorithm 2 - Target {target}', fontsize=20)  # Grotere titel
#     plt.xlabel('Trial Number', fontsize=18)  # Grotere x-as label
#     plt.ylabel('Completion Time', fontsize=18)  # Grotere y-as label
#     plt.xticks(range(1, subset['TrialIndex'].max() + 2), fontsize = 16)  # Alle trial nummers tonen
#     plt.yticks(range(0,14,2), fontsize=16)
#     plt.tight_layout()
#     plt.show()



import numpy as np
import matplotlib.pyplot as plt

# Kleuren per target
kleuren_alg1 = ["#4169E1", "#5A9BD4", "#B0C4DE"]  # Blauwtinten
kleuren_alg2 = ["#2E8B57", "#66CDAA", "#98FB98"]  # Groentinten

bar_width = 0.25

# --- ALGORITHM 1 ---
targets = sorted(stats_alg1['Target'].unique())
fig, ax = plt.subplots(figsize=(16, 5))
for i, target in enumerate(targets):
    subset = stats_alg1[stats_alg1['Target'] == target].sort_values(by="TrialIndex")
    x = np.arange(len(subset)) + i * bar_width - bar_width  # Offset
    ax.bar(x, subset['mean'], yerr=subset['std'], capsize=5, width=bar_width,
           label=f"Target {target}", color=kleuren_alg1[i])

ax.set_title("Mean Normalized Completion Time ± SD - Algorithm 1", fontsize=20)
ax.set_xlabel("Trial Number", fontsize=18)
ax.set_ylabel("Completion Time", fontsize=18)
ax.set_xticks(np.arange(len(subset)))
ax.set_xticklabels([str(i+1) for i in subset['TrialIndex']], fontsize=16)
ax.set_yticks(range(0, 16, 2))
ax.set_ylim(0, 16)  # Zet de y-limiet voor consistentie
ax.tick_params(axis='y', labelsize=16)
ax.legend(loc="upper right", fontsize=12)
plt.tight_layout()
plt.show()

# --- ALGORITHM 2 ---
targets = sorted(stats_alg2['Target'].unique())
fig, ax = plt.subplots(figsize=(16, 5))
for i, target in enumerate(targets):
    subset = stats_alg2[stats_alg2['Target'] == target].sort_values(by="TrialIndex")
    x = np.arange(len(subset)) + i * bar_width - bar_width  # Offset
    ax.bar(x, subset['mean'], yerr=subset['std'], capsize=5, width=bar_width,
           label=f"Target {target}", color=kleuren_alg2[i])

ax.set_title("Mean Normalized Completion Time ± SD - Algorithm 2", fontsize=20)
ax.set_xlabel("Trial Number", fontsize=18)
ax.set_ylabel("Completion Time", fontsize=18)
ax.set_xticks(np.arange(len(subset)))
ax.set_xticklabels([str(i+1) for i in subset['TrialIndex']], fontsize=16)
ax.set_yticks(range(0, 16, 2))
ax.set_ylim(0, 16)  # Zet de y-limiet voor consistentie
ax.tick_params(axis='y', labelsize=16)
ax.legend(loc="upper right", fontsize=12)
plt.tight_layout()
plt.show()


# Gemiddelde van de means per target en algoritme
avg_means_alg1 = stats_alg1.groupby('Target')['mean'].mean().reset_index()
avg_means_alg2 = stats_alg2.groupby('Target')['mean'].mean().reset_index()

print("Gemiddelde Genormaliseerde Duur per Target - Algoritme 1:")
print(avg_means_alg1)

print("\nGemiddelde Genormaliseerde Duur per Target - Algoritme 2:")
print(avg_means_alg2)


# Gemiddelde en standaarddeviatie van de Genormaliseerde Duur per Target
summary_alg1 = stats_alg1.groupby('Target')['mean'].agg(['mean', 'std']).reset_index()
summary_alg2 = stats_alg2.groupby('Target')['mean'].agg(['mean', 'std']).reset_index()

print("Gemiddelde Genormaliseerde Duur per Target - Algoritme 1:")
print(summary_alg1)

print("\nGemiddelde Genormaliseerde Duur per Target - Algoritme 2:")
print(summary_alg2)
