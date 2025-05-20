### CODE IS KLAAR ###

## code laad qualisys data in en berekent de startpunten van de trials
## dit wordt gedaan door het minimum y-waarde van Index te bepalen na de schouderophaal (peak_file)
## waarneer index een threshold overschrijft van de baseline + 10 mm, is dat de start van de trial

## de code bepaald het einpunt van de trial door de afstand van de index naar de targets te bepalen
## wanneer de Index binnen 20 mm van de target is, is dat het eindpunt van de trial
## als dit nooit het geval is, wordt de minimale afstand van de index naar de target bepaald en als eindpunt gekenmerkt

## op basis hiervan wordt de duur van de trials bepaald en geplot


## inladen van alle qualisys data
## inladen van de pieken data

## kijken naar de twee seconden na de schouderophaal en daar de minimale y waarde bepalen
## kijken naar de eerste waarde die daarboven komt
## kijken naar de tijd en frame van die waarde
## het eindpunt bepalen wanneer marker de target bereikt
## duur van de poging bepalen

import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

name = "WouterVisser"
algorithm = "Alg2"
seconds_to_check = 15.0  # seconden na start om te controleren op raakmoment
file_made = True

## qualisys file
qualisys_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
with open(qualisys_file, 'r') as f:
    qualisys_data = json.load(f)
finger_marker = "Index"
target_markers = ["Tar1", "Tar2", "Tar3"]
afstand_drempel_mm = 20.0 # mm
y_threshold = 20.0 # mm
framerate = 128.0  # Hz


## pieken file
peak_file = pd.read_csv(f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\{name}_pieken_qualisys_{algorithm}_0.csv")

## marker data uitlezen
def get_marker_data(name):
    for marker in qualisys_data["Markers"]:
        if marker["Name"] == name:
            return np.array(marker["Parts"][0]["Values"])[:, :3]
    raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

finger_pos = get_marker_data(finger_marker)
targets = {name: get_marker_data(name) for name in target_markers}

# index bepalen uit volledige qualisys data bestand voor pieken
start_peak_idx = np.zeros(len(peak_file), dtype=int)
for i in range(len(peak_file)):
    start_peak_idx[i] = peak_file["time"][i] * framerate

start_trial = np.zeros(len(start_peak_idx), dtype=int)

baseline_threshold = 10.0  # veilige marge in mm

for i in range(len(start_peak_idx)):
    # 2 seconden na de piek
    y_after_peak = finger_pos[start_peak_idx[i]:start_peak_idx[i] + int(2 * framerate), 1]

    # Zoek minimum in de hele 2 seconden (niet beperken tot tweede helft!)
    min_idx_local = np.argmin(y_after_peak)
    min_y = y_after_peak[min_idx_local]

    threshold = min_y + baseline_threshold

    print(f"Piek {i}: minimum y-waarde: {min_y} mm (bij frame {start_peak_idx[i] + min_idx_local})")
    print(f"Threshold: {threshold} mm")

    # Pas NA het minimum zoeken naar threshold crossing
    for k in range(min_idx_local, len(y_after_peak)):
        if y_after_peak[k] > threshold:
            start_trial[i] = start_peak_idx[i] + k
            print(f"Start trial {i}: {start_trial[i]} (frame {start_trial[i]}, tijd {start_trial[i]/framerate:.2f} s)")
            break
    else:
        # Geen crossing gevonden: laatste frame pakken
        start_trial[i] = start_peak_idx[i] + int(2 * framerate) - 1
        print(f"Warning: geen threshold crossing gevonden voor piek {i}, fallback gebruikt.")


print("Start pieken (schouderophaal):", start_peak_idx)
print("Start trials (start reikpoging):", start_trial)


# Maak een tijdas aan in seconden
time = np.arange(finger_pos.shape[0]) / framerate

plt.figure(figsize=(15, 6))
plt.plot(time, finger_pos[:, 1], label='Index vingerpositie (Y)', color='black')

# Rode bolletjes: pieken (schouderophaalmomenten)
plt.scatter(start_peak_idx / framerate, finger_pos[start_peak_idx, 1], color='red', label='Schouderophaal pieken', zorder=5)

# Blauwe bolletjes: start reikpogingen
plt.scatter(start_trial / framerate, finger_pos[start_trial, 1], color='blue', label='Start reikpoging', zorder=5)

plt.xlabel('Tijd (s)')
plt.ylabel('Y-positie Index (mm)')
plt.title('Schouderophaal en start reikpoging Qualisys gedetecteerd')
plt.legend()
plt.grid(True)
plt.show()
## frame waarde is gelijk aan de index.


# Functie om eindpunt te detecteren (eerste moment binnen afstand tot target)
# def detect_eindpunt_na_start(finger_pos, targets, start_frame, afstand_drempel):
#     eind_frames = []

#     for target_name, target_pos in targets.items():
#         afstanden = np.linalg.norm(finger_pos[start_frame:] - target_pos[start_frame:], axis=1)
#         raak_frames = np.where(afstanden < afstand_drempel)[0]

#         if len(raak_frames) > 0:
#             raak_frame = start_frame + raak_frames[0]
#             eind_frames.append(raak_frame)

#     if eind_frames:
#         # Als we raakmomenten gevonden hebben, neem de eerste
#         return min(eind_frames)
#     else:
#     # Geen raakmoment: zoek hoogste Y-positie binnen 7 seconden na start
#         frames_to_check = int(12 * framerate)  # maximaal 7 seconden na start
#         y_pos_after_start = finger_pos[start_frame:start_frame + frames_to_check, 1]  # Y-waarden binnen tijdsvenster
#         print(f"Y-waarden rond piek 17 (start_frame {start_frame}): {y_pos_after_start}")
#         max_idx_local = np.argmax(y_pos_after_start)
#         eind_frame = start_frame + max_idx_local
#         return eind_frame


############ probeersel emy ###############

# def detect_eindpunt_na_start(finger_pos, targets, start_frame, afstand_drempel):
#     eind_frames = []
#     frames_to_check = int(7 * framerate)  # maximaal 7 seconden na start

#     for target_name, target_pos in targets.items():
#         # Let op: we beperken tot 7 seconden venster
#         afstanden = np.linalg.norm(finger_pos[start_frame:start_frame + frames_to_check] - 
#                                    target_pos[start_frame:start_frame + frames_to_check], axis=1)
#         raak_frames = np.where(afstanden < afstand_drempel)[0]

#         if len(raak_frames) > 0:
#             raak_frame = start_frame + raak_frames[0]
#             eind_frames.append(raak_frame)

#     if eind_frames:
#         # Als er raakmomenten gevonden zijn, neem de vroegste
#         return min(eind_frames)
#     else:
#         # Geen raakmoment: zoek de minimale afstand binnen het 7s venster
#         min_afstanden_per_target = []
#         for target_name, target_pos in targets.items():
#             afstanden = np.linalg.norm(finger_pos[start_frame:start_frame + frames_to_check] - 
#                                        target_pos[start_frame:start_frame + frames_to_check], axis=1)
#             min_idx_local = np.argmin(afstanden)
#             eind_frame = start_frame + min_idx_local
#             min_afstanden_per_target.append(eind_frame)

#         # Neem het frame waarbij je het dichtst bij een van de targets bent gekomen
#         return min(min_afstanden_per_target)

def detect_eindpunt_na_start(finger_pos, targets, start_frame, afstand_drempel):
    frames_to_check = int(seconds_to_check * framerate)  # maximaal 7 seconden na start
    eind_frames = []

    for target_name, target_pos in targets.items():
        afstanden = np.linalg.norm(finger_pos[start_frame:start_frame + frames_to_check] - 
                                   target_pos[start_frame:start_frame + frames_to_check], axis=1)
        raak_frames = np.where(afstanden < afstand_drempel)[0]

        if len(raak_frames) > 0:
            raak_frame = start_frame + raak_frames[0]
            eind_frames.append(raak_frame)

    if eind_frames:
        # Als er raakmomenten gevonden zijn, neem de vroegste
        return min(eind_frames)
    else:
        # Geen raakmoment: zoek globaal de dichtstbijzijnde afstand tot een van de targets
        min_afstand = np.inf
        min_frame = start_frame

        for frame_rel in range(frames_to_check):
            afstand_per_target = []
            for target_name, target_pos in targets.items():
                afstand = np.linalg.norm(finger_pos[start_frame + frame_rel] - target_pos[start_frame + frame_rel])
                afstand_per_target.append(afstand)

            min_afstand_frame = min(afstand_per_target)

            if min_afstand_frame < min_afstand:
                min_afstand = min_afstand_frame
                min_frame = start_frame + frame_rel

        return min_frame


# Arrays om eindframes en duur per trial op te slaan
end_trial = np.full(len(start_trial), np.nan)  # default NaN als niet gevonden
durations = np.full(len(start_trial), np.nan)

# Over alle trials
for i in range(len(start_trial)):
    eind_frame = detect_eindpunt_na_start(finger_pos, targets, start_trial[i], afstand_drempel_mm)
    
    if eind_frame is not None:
        end_trial[i] = eind_frame
        durations[i] = (end_trial[i] - start_trial[i]) / framerate
    else:
        print(f"⚠️  Geen eindpunt gevonden voor trial {i}.")

# Resultaat tonen
for i in range(len(start_trial)):
    print(f"Trial {i}: start = {start_trial[i]} ({start_trial[i]/framerate:.2f} s), "
          f"eind = {end_trial[i] if not np.isnan(end_trial[i]) else 'niet gevonden'} "
          f"({end_trial[i]/framerate:.2f} s)" if not np.isnan(end_trial[i]) else "", 
          f"duur = {durations[i]:.3f} s" if not np.isnan(durations[i]) else "duur = onbekend")


# === Bepaal per trial de juiste target ===
trial_targets = []

frames_to_check = int(seconds_to_check * framerate)  # maximaal 7 seconden na start

for i in range(len(start_trial)):
    min_distance = np.inf
    closest_target = None

    for target_name, target_pos in targets.items():
        # Bereken afstand binnen 7 seconden na start
        afstanden = np.linalg.norm(
            finger_pos[int(start_trial[i]):int(start_trial[i]) + frames_to_check] -
            target_pos[int(start_trial[i]):int(start_trial[i]) + frames_to_check],
            axis=1
        )
        min_dist = np.min(afstanden)
        if min_dist < min_distance:
            min_distance = min_dist
            closest_target = target_name

    trial_targets.append(closest_target)

# === Nu plotten met zwarte lijn en gekleurde achtergrond ===
fig, ax = plt.subplots(figsize=(12, 6))

tijd = np.arange(finger_pos.shape[0]) / framerate

# Kleuren per target
kleuren = plt.cm.get_cmap('tab10', len(targets))
target_kleur_idx = {target_name: idx for idx, target_name in enumerate(targets.keys())}

# Om dubbele legend labels te voorkomen
already_plotted_targets = set()

# Eerst achtergrondkleuren tekenen
for i in range(len(start_trial)):
    if not np.isnan(end_trial[i]):
        start = start_trial[i] / framerate
        eind = end_trial[i] / framerate

        target_name = trial_targets[i]
        kleur = kleuren(target_kleur_idx[target_name])

        label = target_name if target_name not in already_plotted_targets else None
        ax.axvspan(start, eind, color=kleur, alpha=0.3, label=label)
        already_plotted_targets.add(target_name)

# Daarna de afstandslijn tekenen (zodat die boven de vlakken ligt)
# Bijvoorbeeld afstand tot "juiste" target of gemiddelde afstand
# Hier teken ik gewoon de afstand tot de juiste target tijdens de hele trial

for i in range(len(start_trial)):
    if not np.isnan(end_trial[i]):
        start_idx = int(start_trial[i])
        eind_idx = int(end_trial[i])

        target_name = trial_targets[i]
        afstanden = np.linalg.norm(finger_pos - targets[target_name], axis=1)

        tijd_filtered = tijd[start_idx:eind_idx]
        afstand_filtered = afstanden[start_idx:eind_idx]

        ax.plot(tijd_filtered, afstand_filtered, color='black')

ax.set_xlabel("Tijd (s)")
ax.set_ylabel("Afstand tot target (mm)")
ax.set_title("Afstand Index naar Target met gekleurde targetvlakken")
ax.grid(True)
ax.legend(title="Target")
plt.tight_layout()
plt.show()






# Unieke targets ophalen
unieke_targets = sorted(set(trial_targets))  # ['Tar1', 'Tar2', 'Tar3']

# Subplots instellen (onder elkaar)
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Kleur per target
colors = {'Tar1': 'red', 'Tar2': 'green', 'Tar3': 'blue'}

# Trials per target ordenen
for i, target in enumerate(unieke_targets):
    ax = axs[i]
    
    # Vind trials die bij deze target horen en sorteer ze
    trial_idx = [j for j, t in enumerate(trial_targets) if t == target]
    trial_durations = durations[trial_idx]

    # Staafdiagram plotten (de x-as moet 1 t/m 10 zijn per target)
    ax.bar(np.arange(1, len(trial_durations) + 1), trial_durations, color=colors[target], edgecolor='black')
    
    # Titel en labels instellen
    ax.set_title(f'Trial duur voor {target}')
    ax.set_xlabel('Trial nummer')
    if i == 0:
        ax.set_ylabel('Duur (seconden)')
    
    # Zet een grid voor de Y-as
    ax.grid(True, axis='y')

# Titels en lay-out verbeteren
fig.suptitle('Duur van trials per target', fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Zorg ervoor dat de titel goed gepositioneerd is
plt.show()




# === Staafdiagram voor alle trials in originele volgorde, met kleuren per target ===
plt.figure(figsize=(10, 6))

# Stel een kleurmap in voor de drie targets
colors = {'Tar1': 'red', 'Tar2': 'green', 'Tar3': 'blue'}

# Maak een lijst van kleuren op basis van trial_targets
trial_colors = [colors[target] for target in trial_targets]

# Staafdiagram plotten met de duur van elke trial en de kleur per target
plt.bar(np.arange(1, len(durations) + 1), durations, color=trial_colors, edgecolor='black')

# Titels en labels instellen
plt.title('Duur van alle trials per target', fontsize=16)
plt.xlabel('Trial nummer')
plt.ylabel('Duur (seconden)')
plt.xticks(np.arange(1, len(durations) + 1))  # Zorg ervoor dat alle trial nummers op de x-as komen
plt.grid(True, axis='y')

# Maak een legend voor de targets
import matplotlib.patches as mpatches
handles = [mpatches.Patch(color=color, label=target) for target, color in colors.items()]
plt.legend(handles=handles, title='Target')

# Weergeven van de plot
plt.tight_layout()
plt.show()










####### einde probeersel emy ###############

# Arrays om eindframes en duur per trial op te slaan
end_trial = np.full(len(start_trial), np.nan)  # default NaN als niet gevonden
durations = np.full(len(start_trial), np.nan)

# Over alle trials
for i in range(len(start_trial)):
    eindframes_per_target = []
    for target_name in target_markers:
        # target_pos = targets[target_name]
        eind_frame = detect_eindpunt_na_start(finger_pos, targets, start_trial[i], afstand_drempel_mm)
        if eind_frame is not None:
            eindframes_per_target.append(eind_frame)
    
    if eindframes_per_target:
        # Kies het vroegste eindmoment als er meerdere targets worden geraakt
        end_trial[i] = np.min(eindframes_per_target)
        durations[i] = (end_trial[i] - start_trial[i]) / framerate
    else:
        print(f"⚠️  Geen raakmoment gevonden voor trial {i}.")

# Resultaat tonen
for i in range(len(start_trial)):
    print(f"Trial {i}: start = {start_trial[i]} ({start_trial[i]/framerate:.2f} s), "
          f"eind = {end_trial[i] if not np.isnan(end_trial[i]) else 'niet gevonden'} "
          f"({end_trial[i]/framerate:.2f} s)" if not np.isnan(end_trial[i]) else "", 
          f"duur = {durations[i]:.3f} s" if not np.isnan(durations[i]) else "duur = onbekend")


# Maak een tijdas aan in seconden
time = np.arange(finger_pos.shape[0]) / framerate

# === Plotten van start en eind van reikpogingen ===
plt.figure(figsize=(15, 6))
plt.plot(time, finger_pos[:, 1], label='Index vingerpositie (Y)', color='black')
plt.plot(time, finger_pos[:, 0], label='X-positie', color='orange')
plt.plot(time, finger_pos[:, 2], label='Z-positie', color='purple')

# Rode bolletjes: pieken (schouderophaalmomenten)
plt.scatter(start_peak_idx / framerate, finger_pos[start_peak_idx, 1], color='blue', label='Schouderophaal pieken', zorder=5)

# Blauwe bolletjes: start reikpogingen
plt.scatter(start_trial / framerate, finger_pos[start_trial, 1], color='green', label='Start reikpoging', zorder=5)

# Groene bolletjes: eind reikpogingen
valid_end_trials = ~np.isnan(end_trial)
plt.scatter(end_trial[valid_end_trials] / framerate, finger_pos[end_trial[valid_end_trials].astype(int), 1], 
            color='red', label='Eind reikpoging', zorder=5)

plt.xlabel('Tijd (s)')
plt.ylabel('Y-positie (mm)')
plt.title('Schouderophaal, start en einde reikpogingen gedetecteerd')
plt.legend()
plt.grid(True)
plt.show()


import csv

# Stel de naam van het bestand in (pas dit aan naar wens)
trial_file= f"{name}_trials_qualisys_{algorithm}.csv"

# Maak een lijst van dictionaries met de informatie voor elke trial
trial_data = []

for i in range(len(start_trial)):
    start_frame = start_trial[i]  # Startframe
    eind_frame = end_trial[i] if not np.isnan(end_trial[i]) else np.nan  # Eindframe
    start_time = start_trial[i] / framerate  # Starttijd in seconden
    end_time = end_trial[i] / framerate if not np.isnan(end_trial[i]) else np.nan  # Eindtijd in seconden
    trial_duration = durations[i]  # Duur van de trial
    target_name = trial_targets[i]  # Het bijbehorende target voor deze trial

    # Voeg een dictionary toe met de gegevens van de trial
    trial_data.append({
        'Trial': i + 1,
        'Startframe': start_frame,
        'Eindframe': eind_frame,
        'Starttijd (s)': start_time,
        'Eindtijd (s)': end_time,
        'Target': target_name,
        'Duur (s)': trial_duration
    })

# Schrijf de data naar een CSV-bestand
if file_made == False:
    with open(trial_file, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=['Trial', 'Startframe', 'Eindframe', 'Starttijd (s)', 'Eindtijd (s)', 'Target', 'Duur (s)'])
        writer.writeheader()
        writer.writerows(trial_data)

    print(f"CSV bestand '{trial_file}' is succesvol opgeslagen!")







# import json
# import numpy as np
# import matplotlib.pyplot as plt

# # === Instellingen ===
# json_path = "D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\Alg1.json"
# vinger_marker = "Index"
# target_markers = ["Tar1", "Tar2", "Tar3"]
# afstand_drempel_mm = 20.0
# y_start_drempel_boven_min_mm = 30.0 # 33.0 voor Nullmeting Anna
# framerate = 128.0  # Hz

# # === JSON inladen ===
# with open(json_path, "r") as f:
#     data = json.load(f)

# # === Markerdata extraheren ===
# def get_marker_data(name):
#     for marker in data["Markers"]:
#         if marker["Name"] == name:
#             return np.array(marker["Parts"][0]["Values"])[:, :3]
#     raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

# finger_pos = get_marker_data(vinger_marker)
# targets = {name: get_marker_data(name) for name in target_markers}

# # === Functie: detecteer reiken op basis van Y-hoogte én aanraking ===
# def detect_reiken_op_basis_van_y(finger_pos, target_pos, afstand_drempel, y_drempel_boven_min, fps):
#     afstanden = np.linalg.norm(finger_pos - target_pos, axis=1)
#     binnen = afstanden < afstand_drempel
#     raak_frames = np.where(np.diff(binnen.astype(int)) == 1)[0] + 1  # raakmomenten

#     y_min = np.min(finger_pos[:, 1])
#     y_drempel = y_min + y_drempel_boven_min

#     pogingen = []
#     laatste_start_tijd = -np.inf

#     for raak_frame in raak_frames:
#         # Zoek het laatste frame vóór de aanraking waar Y < drempel
#         voor_moment = np.where(finger_pos[:raak_frame, 1] < y_drempel)[0]
#         if len(voor_moment) == 0:
#             continue  # geen geldig startmoment gevonden
#         start_frame = voor_moment[-1]
#         duur = (raak_frame - start_frame) / fps
#         start_tijd = start_frame / fps
#         raak_tijd = raak_frame / fps

#         # Vermijd dubbele pogingen (minimaal 1s tussenpoging)
#         if start_tijd - laatste_start_tijd >= 1.0:
#             pogingen.append((start_frame, raak_frame, start_tijd, raak_tijd, duur))
#             laatste_start_tijd = start_tijd

#     return pogingen

# # === Analyse uitvoeren ===
# print("\n📊 Verbeterde reiktaakanalyse (Y-hoogte als startcriterium):")
# for target_name, target_pos in targets.items():
#     pogingen = detect_reiken_op_basis_van_y(
#         finger_pos, target_pos, afstand_drempel_mm, y_start_drempel_boven_min_mm, framerate
#     )

#     print(f"\n🎯 Target: {target_name}")
#     if not pogingen:
#         print("  Geen pogingen gevonden.")
#     for i, (start, raak, t_start, t_raak, duur) in enumerate(pogingen, 1):
#         print(f"  ▸ Poging {i}:")
#         print(f"     Start: frame {start}, tijd {t_start:.3f} s")
#         print(f"     Raak : frame {raak}, tijd {t_raak:.3f} s")
#         print(f"     Duur : {duur:.3f} s")


# # === Staafdiagrammen per target in subplots ===
# fig, axs = plt.subplots(len(targets), 1, figsize=(8, 3 * len(targets)), sharex=True)

# if len(targets) == 1:
#     axs = [axs]  # Zorg dat axs altijd een lijst is, ook als er maar 1 subplot is

# for ax, (target_name, target_pos) in zip(axs, targets.items()):
#     pogingen = detect_reiken_op_basis_van_y(
#         finger_pos, target_pos, afstand_drempel_mm, y_start_drempel_boven_min_mm, framerate
#     )

#     duren = [duur for _, _, _, _, duur in pogingen]
#     pogingsnummers = list(range(1, len(duren) + 1))

#     ax.bar(pogingsnummers, duren, color='skyblue')
#     ax.set_title(f"Target: {target_name}")
#     ax.set_ylabel("Duur (s)")
#     ax.grid(True)

# axs[-1].set_xlabel("Poging nummer")
# plt.tight_layout()
# plt.show()


# # === Positieplot met gekleurde reikpogingen per target ===
# fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

# tijd = np.arange(finger_pos.shape[0]) / framerate
# labels = ['X-positie', 'Y-positie', 'Z-positie']
# kleuren = plt.cm.get_cmap('tab10', len(targets))  # unieke kleur per target

# for i in range(3):  # Voor X, Y, Z
#     axs[i].plot(tijd, finger_pos[:, i], label=labels[i], color='black')
#     axs[i].set_ylabel(labels[i] + " (mm)")
#     axs[i].grid(True)

# # Voeg gekleurde vlakken toe voor elke poging per target
# for idx, (target_name, target_pos) in enumerate(targets.items()):
#     pogingen = detect_reiken_op_basis_van_y(
#         finger_pos, target_pos, afstand_drempel_mm, y_start_drempel_boven_min_mm, framerate
#     )
#     kleur = kleuren(idx)
#     for start_frame, raak_frame, t_start, t_raak, _ in pogingen:
#         for ax in axs:
#             ax.axvspan(t_start, t_raak, color=kleur, alpha=0.3)
#     axs[0].plot([], [], color=kleur, label=target_name)  # Voor legenda

# axs[-1].set_xlabel("Tijd (s)")
# axs[0].legend(title="Target")
# plt.suptitle("Index-positie met aangeduide reikpogingen")
# plt.tight_layout(rect=[0, 0.03, 1, 0.95])
# plt.show()

# ##########

# # === Gecombineerde plot van X, Y en Z met gemarkeerde reikpogingen ===
# fig, ax = plt.subplots(figsize=(12, 6))

# tijd = np.arange(finger_pos.shape[0]) / framerate

# # Plot X, Y en Z
# ax.plot(tijd, finger_pos[:, 0], label='X-positie', color='red')
# ax.plot(tijd, finger_pos[:, 1], label='Y-positie', color='green')
# ax.plot(tijd, finger_pos[:, 2], label='Z-positie', color='blue')

# # Reikpogingen per target kleuren
# kleuren = plt.cm.get_cmap('tab10', len(targets))

# for idx, (target_name, target_pos) in enumerate(targets.items()):
#     pogingen = detect_reiken_op_basis_van_y(
#         finger_pos, target_pos, afstand_drempel_mm, y_start_drempel_boven_min_mm, framerate
#     )
#     kleur = kleuren(idx)
#     for start_frame, raak_frame, t_start, t_raak, _ in pogingen:
#         ax.axvspan(t_start, t_raak, color=kleur, alpha=0.3, label=target_name if t_start == pogingen[0][2] else "")

# ax.set_xlabel("Tijd (s)")
# ax.set_ylabel("Positie (mm)")
# ax.set_title("Index X, Y en Z positie over tijd met gemarkeerde reikpogingen")
# ax.grid(True)
# ax.legend(title="Coördinaten / Targets")
# plt.tight_layout()
# plt.show()
