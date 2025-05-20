import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

name = "WouterVisser"
algoritme = "Alg1"

# === Stap 1: CSV inlezen ===
# Vervang 'pad_naar_bestand.csv' door het juiste pad naar je bestand
csv_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_{algoritme}.csv"

df = pd.read_csv(csv_file)

# Controleer of de kolommen aanwezig zijn
assert 'Duur (s)' in df.columns and 'Target' in df.columns, "CSV moet kolommen 'Duur (s)' en 'Target' bevatten."

# Extract data
durations = df['Duur (s)'].values
trial_targets = df['Target'].values

# === Unieke targets ophalen ===
unieke_targets = sorted(set(trial_targets))  # Bijvoorbeeld ['Tar1', 'Tar2', 'Tar3']

# === Subplots instellen ===
fig, axs = plt.subplots(len(unieke_targets), 1, figsize=(10, 4 * len(unieke_targets)), sharex=True)

# Kleur per target (pas aan indien meer dan 3 targets)
kleurenlijst = ['red', 'green', 'blue', 'orange', 'purple', 'cyan']
colors = {target: kleurenlijst[i % len(kleurenlijst)] for i, target in enumerate(unieke_targets)}

# Trials per target plotten
for i, target in enumerate(unieke_targets):
    ax = axs[i] if len(unieke_targets) > 1 else axs  # Bij 1 target is axs geen lijst
    
    trial_idx = df[df['Target'] == target].index
    trial_durations = df.loc[trial_idx, 'Duur (s)'].values

    ax.bar(np.arange(1, len(trial_durations) + 1), trial_durations, color=colors[target], edgecolor='black')
    ax.set_title(f'Trial duur voor {target}, {algoritme}')
    ax.set_xlabel('Trial nummer')
    if i == 0:
        ax.set_ylabel('Duur (seconden)')
    ax.grid(True, axis='y')

fig.suptitle('Duur van trials per target', fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()

# === Alle trials in originele volgorde ===
plt.figure(figsize=(10, 6))

# Kleur per trial
trial_colors = [colors[target] for target in trial_targets]

plt.bar(np.arange(1, len(durations) + 1), durations, color=trial_colors, edgecolor='black')
plt.title('Duur van alle trials per target', fontsize=16)
plt.xlabel('Trial nummer')
plt.ylabel('Duur (seconden)')
plt.xticks(np.arange(1, len(durations) + 1))
plt.grid(True, axis='y')

# Legend toevoegen
handles = [mpatches.Patch(color=color, label=target) for target, color in colors.items()]
plt.legend(handles=handles, title='Target')

plt.tight_layout()
plt.show()
