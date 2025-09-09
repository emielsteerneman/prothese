### GESCHIKT IS VOOR NULL METINGEN ###
### CODE IS KLAAR ###

## code laad qualisys data in en bepaald waar de trials zijn door de pieken van y_waarde Index te selecteren
## dit wordt gedaan door de pieken te selecteren in de grafiek
## er wordt in 1 seconde voor de piek gekeken naar de y-waarde van de index
## de minimale waarde wordt bepaald en wanneer de index daarboven komt met 10mm, is dat het startpunt van de trial
## het eindpunt van de trial wordt bepaald door de afstand van de index naar de targets te bepalen
## wanneer de index binnen 20 mm van de target is, is dat het eindpunt van de trial
## als dit nooit het geval is, wordt de minimale afstand van de index naar de target bepaald en als eindpunt gekenmerkt
## op basis hiervan wordt de duur van de trials bepaald en geplot
## alle relevant gegevens worden opgeslagen in een csv bestand

import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

name = "BasvanderKaaden"
algorithm = "Null"
peaks_selected = True # als de pieken al geselecteerd zijn, zet dit op True
seconds_before_peak = 1.5 # seconden voor de piek om naar te kijken
file_made = True

## qualisys file
qualisys_file = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
with open(qualisys_file, 'r') as f:
    qualisys_data = json.load(f)
finger_marker = "Index"
target_markers = ["Tar1", "Tar2", "Tar3"]
afstand_drempel_mm = 20.0 # mm
framerate = 128.0  # Hz


## marker data uitlezen
def get_marker_data(name):
    for marker in qualisys_data["Markers"]:
        if marker["Name"] == name:
            return np.array(marker["Parts"][0]["Values"])[:, :3]
    raise ValueError(f"Marker '{name}' niet gevonden in JSON.")

finger_pos = get_marker_data(finger_marker)
targets = {name: get_marker_data(name) for name in target_markers}
qualisys_time = np.arange(finger_pos.shape[0]) / framerate

plt.plot(qualisys_time, finger_pos[:, 1], label='Index vingerpositie (Y)', color='black')
plt.xlabel('Tijd (s)')
plt.ylabel('Y-positie Index (mm)')
plt.title('Y-positie van de index vinger')
plt.legend()
plt.grid(True)
plt.show()



### selecteer pieken en sla data op
if not peaks_selected:
    qualisys_peaks = []

    fig, ax = plt.subplots()
    ax.plot(qualisys_time, finger_pos[:, 1], label="Index vinger", color="red")
    ax.set_title("Klik op de pieken van de Index on de qualisys data om de coördinaten op te slaan")
    line, = ax.plot(qualisys_time, finger_pos[:, 1], color='blue', picker=5)  # zorg ervoor dat de line picker heeft

    # Klikfunctie voor pieken selecteren
    def onclick(event):
        # Zorg ervoor dat de klik op de lijn wordt geregistreerd
        if event.artist == line:
            # Verkrijg de locatie van de klik
            mouse_event = event.mouseevent
            xdata = mouse_event.xdata
            ydata = mouse_event.ydata

            # Zoek het dichtstbijzijnde tijdstip
            idx = np.argmin(np.abs(qualisys_time - xdata))
            
            # Voeg de piek toe aan de lijst met geselecteerde pieken
            qualisys_peaks.append((qualisys_time[idx], finger_pos[idx, 1]))
            
            # Markeer de geselecteerde piek op de grafiek
            ax.plot(qualisys_time[idx], finger_pos[idx, 1], 'ro')  # voeg de rode stip toe
            fig.canvas.draw()  # update de grafiek na de klik

    # Koppel de klikfunctie aan het canvas
    cid = fig.canvas.mpl_connect("pick_event", onclick)

    # Toon de grafiek
    plt.show()

    # Print de geselecteerde pieken
    print("Geselecteerde pieken:")
    for qualisys_time, index_value in qualisys_peaks:
        print(f"  Piek: tijd {qualisys_time:.4f} s , Index {index_value:.2f} mm")

    # Opslaan van de geselecteerde pieken naar een CSV-bestand
    df = pd.DataFrame(qualisys_peaks, columns=['time', 'Index'])
    df.to_csv(f'{name}_pieken_qualisys_{algorithm}.csv', index=False)

    print(f"Opgeslagen als '{name}_pieken_qualisys_{algorithm}.csv'")




### zoek startpunt van de trial
# Parameters
baseline_threshold = 10.0  # veilige marge in mm

# Inladen van piekdata
Null_peak_file = pd.read_csv(f'{name}_pieken_qualisys_{algorithm}.csv')

start_trial = []


# Loopen door alle pieken
for i in range(len(Null_peak_file)):
    # Tijd van de piek
    peak_time = Null_peak_file["time"][i]
    
    # Zoek het index van de piek in de tijdstempels (tijd * framerate en rond af naar beneden)
    peak_idx = int(peak_time * framerate)

    # Zorg ervoor dat we binnen het bereik van de array blijven
    if peak_idx == 0:
        continue  # Als de piek aan het begin van de data is, slaan we het over

    # Verkrijg de y-waarde van de vingerpositie vlak voor de piek (een seconde ervoor)
    time_before_peak = int(peak_time * framerate) - int(seconds_before_peak * framerate)  # 1 seconde ervoor
    if time_before_peak < 0:
        time_before_peak = 0  # Om negatieve indices te voorkomen

    y_value_before_peak = finger_pos[time_before_peak, 1]

    # Vind de minimale y-waarde in deze periode (1 seconde voor de piek)
    min_idx_local = np.argmin(finger_pos[time_before_peak:peak_idx, 1]) + time_before_peak
    min_y = finger_pos[min_idx_local, 1]

    # Drempelwaarde
    threshold = min_y + baseline_threshold

    for k in range(min_idx_local, len(finger_pos)):
        if finger_pos[k, 1] > threshold:
            # We hebben de drempel overschreden
            time_above_threshold = qualisys_time[k]  # tijd waarop de drempel wordt overschreden
            print(f"Piek {i}: De drempel wordt overschreden bij tijd {time_above_threshold:.4f} s (y-waarde {finger_pos[k, 1]:.2f} mm)")
            start_trial.append({
                'Piek': i,
                'Minimum_y_waarde': min_y,
                'Tijd_minimum_y': qualisys_time[min_idx_local],
                'Threshold': threshold,
                'Tijd_overschreden': time_above_threshold,
                'y_waarde_overschreden': finger_pos[k, 1]
            })
            break  # Stop zodra de drempel is overschreden

        

    # Resultaat printen
    print(f"Piek {i}: minimum y-waarde: {min_y} mm (bij frame {min_idx_local}), drempelwaarde: {threshold} mm")

plt.plot(qualisys_time, finger_pos[:, 1], label='Index vingerpositie (Y)', color='black')
plt.plot(Null_peak_file["time"], Null_peak_file["Index"], 'bo', label='Geselecteerde pieken')
# Plot de drempeloverschrijding (laatste toegevoegde resultaat in start_trial)
for trial in start_trial:
    time_above_threshold = trial['Tijd_overschreden']
    y_above_threshold = trial['y_waarde_overschreden']
    plt.plot(time_above_threshold, y_above_threshold, 'go', label=f'Drempel overschreden (Piek {trial["Piek"]})')
plt.xlabel('Tijd (s)')
plt.ylabel('Y-positie (mm)')
plt.title('Y-positie van de index vinger met geselecteerde pieken')
plt.grid(True)
plt.show()







## nu gaan we kijken naar de afstand tussen index en target en wanneer deze kleiner is dan 20mm. dit zal het eindpunt van de trial zijn
## als dit nooit het geval is, wordt de minimale afstand van de index naar de target bepaald en als eindpunt gekenmerkt

def detect_eindpunt_na_start(finger_pos, targets, start_frame, afstand_drempel):
    frames_to_check = int(2 * framerate)  # maximaal 2 seconden na start
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
    start_frame = int(start_trial[i]['Tijd_overschreden'] * framerate)
    eind_frame = detect_eindpunt_na_start(finger_pos, targets, start_frame, afstand_drempel_mm)

    if eind_frame is not None:
        end_trial[i] = eind_frame
        durations[i] = (eind_frame - start_frame) / framerate
    else:
        print(f"⚠️  Geen eindpunt gevonden voor trial {i}.")


# Resultaat tonen
for i in range(len(start_trial)):
    start_time = start_trial[i]['Tijd_overschreden']
    print(f"Trial {i}: start = frame {int(start_time * framerate)} ({start_time:.2f} s), "
        f"eind = {int(end_trial[i]) if not np.isnan(end_trial[i]) else 'niet gevonden'} "
        f"({end_trial[i]/framerate:.2f} s)" if not np.isnan(end_trial[i]) else "",
        f"duur = {durations[i]:.3f} s" if not np.isnan(durations[i]) else "duur = onbekend")



    # print(f"Trial {i}: start = {start_trial[i]} ({start_trial[i]/framerate:.2f} s), "
    #       f"eind = {end_trial[i] if not np.isnan(end_trial[i]) else 'niet gevonden'} "
    #       f"({end_trial[i]/framerate:.2f} s)" if not np.isnan(end_trial[i]) else "", 
    #       f"duur = {durations[i]:.3f} s" if not np.isnan(durations[i]) else "duur = onbekend")


# # === Bepaal per trial de juiste target ===
trial_targets = []

afstand_drempel_detectie = 20.0  # mm

for i in range(len(start_trial)):
    if np.isnan(end_trial[i]):
        trial_targets.append(None)
        continue

    start_frame = int(start_trial[i]['Tijd_overschreden'] * framerate)
    eind_frame = int(end_trial[i])
    max_frames = eind_frame - start_frame

    raakmomenten = {}

    for target_name, target_pos in targets.items():
        afstanden = np.linalg.norm(
            finger_pos[start_frame:eind_frame] -
            target_pos[start_frame:eind_frame],
            axis=1
        )
        raak_idx = np.where(afstanden < afstand_drempel_detectie)[0]
        if len(raak_idx) > 0:
            raakmomenten[target_name] = raak_idx[0]

    if raakmomenten:
        # Kies target met vroegste raakmoment tijdens trial
        beste_target = min(raakmomenten, key=raakmomenten.get)
    else:
        # Fallback: target dichtst bij eindpositie
        min_afstanden = {}
        for target_name, target_pos in targets.items():
            afstand = np.linalg.norm(finger_pos[eind_frame] - target_pos[eind_frame])
            min_afstanden[target_name] = afstand
        beste_target = min(min_afstanden, key=min_afstanden.get)

    trial_targets.append(beste_target)









# trial_targets = []

# frames_to_check = int(1 * framerate)  # maximaal 7 seconden na start

# for i in range(len(start_trial)):
#     min_distance = np.inf
#     closest_target = None
#     start_frame = int(start_trial[i]['Tijd_overschreden'] * framerate)

#     for target_name, target_pos in targets.items():
#         # Bereken afstand binnen 7 seconden na start
#         afstanden = np.linalg.norm(
#             finger_pos[start_frame:start_frame + frames_to_check] -
#             target_pos[start_frame:start_frame + frames_to_check],
#             axis=1
#         )
#         min_dist = np.min(afstanden)
#         if min_dist < min_distance:
#             min_distance = min_dist
#             closest_target = target_name

#     trial_targets.append(closest_target)


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
        start = start_trial[i]['Tijd_overschreden']
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
        start_idx = int(start_trial[i]['Tijd_overschreden'] * framerate)
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



fig, ax = plt.subplots(figsize=(12, 6))

# Y-positie van de indexvinger over de tijd
ax.plot(qualisys_time, finger_pos[:, 1], color='black', label='Y-positie indexvinger')

# Piekmomenten (blauwe bolletjes)
ax.plot(Null_peak_file["time"], Null_peak_file["Index"], 'bo', label='Pieken')

# Start- en eindpunten (groen en rood)
for i, trial in enumerate(start_trial):
    start_time = trial['Tijd_overschreden']
    start_y = trial['y_waarde_overschreden']
    ax.plot(start_time, start_y, 'go', label='Startpunt' if i == 0 else "")  # Alleen eerste keer label

    if not np.isnan(end_trial[i]):
        eind_frame = int(end_trial[i])
        eind_time = qualisys_time[eind_frame]
        eind_y = finger_pos[eind_frame, 1]
        ax.plot(eind_time, eind_y, 'ro', label='Eindpunt' if i == 0 else "")  # Alleen eerste keer label

ax.set_xlabel('Tijd (s)')
ax.set_ylabel('Y-positie (mm)')
ax.set_title('Y-positie indexvinger met piek-, start- en eindpunten')
ax.grid(True)
ax.legend()
plt.tight_layout()
plt.show()

print(start_trial[0], type(start_trial[0]))


import csv

# Stel de naam van het bestand in (pas dit aan naar wens)
trial_file = f"{name}_trials_qualisys_{algorithm}.csv"

# Maak een lijst van dictionaries met de informatie voor elke trial
trial_data = []

for i in range(len(start_trial)):
    # Gebruik de tijd waarop de drempel werd overschreden voor starttijd en frame
    start_time = start_trial[i]['Tijd_overschreden']
    start_frame = int(start_time * framerate)

    eind_frame = end_trial[i] if not np.isnan(end_trial[i]) else np.nan
    end_time = end_trial[i] / framerate if not np.isnan(end_trial[i]) else np.nan
    trial_duration = durations[i]
    target_name = trial_targets[i]

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



