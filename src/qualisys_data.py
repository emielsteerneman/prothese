import json
import numpy as np
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


# === Instellingen ===
json_pad = "D:\BMT\Master\Thesis\Arduino\prothese\qualisys_data\mest_finger.json"  # vervang door jouw pad
marker_1_naam = "target"
marker_2_naam = "finger"
afstand_drempel_mm = 20  # 20 mm = 2 cm
sample_frequentie = 128  # Hz, pas aan als jouw data anders is




# === JSON inladen ===
with open(json_pad, "r") as f:
    data = json.load(f)

# === Markerdata extraheren ===
def extract_marker(marker_name):
    for marker in data["Markers"]:
        if marker["Name"] == marker_name:
            return np.array(marker["Parts"][0]["Values"])
    raise ValueError(f"Marker '{marker_name}' niet gevonden in JSON.")

marker_1 = extract_marker(marker_1_naam)[:, :3]
marker_2 = extract_marker(marker_2_naam)[:, :3]

# === Afstanden berekenen ===
vector_verschil = marker_2 - marker_1  # (N, 3)
euclidische_afstand = np.linalg.norm(vector_verschil, axis=1)

# === Aanraakmoment detecteren ===
aanraak_indexen = np.where(euclidische_afstand <= afstand_drempel_mm)[0]

if aanraak_indexen.size == 0:
    print("❌ Geen aanraking binnen 2 cm gedetecteerd.")
else:
    i = aanraak_indexen[0]
    tijd = i / sample_frequentie
    dx, dy, dz = vector_verschil[i]
    totale_afstand = euclidische_afstand[i]

    print(f"✅ Aanraking op frame {i} ({tijd:.3f} s):")
    print(f"   ➤ Totale afstand: {totale_afstand:.2f} mm")
    print(f"   ➤ ΔX: {dx:.2f} mm")
    print(f"   ➤ ΔY: {dy:.2f} mm")
    print(f"   ➤ ΔZ: {dz:.2f} mm")

# === 3D plot van de bewegende marker met tijdskleur en kleurlegenda ===

framerate = 128  # Hz
n_frames = marker_2.shape[0]
colors = cm.viridis(np.linspace(0, 1, n_frames))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Bewegende marker tekenen (kleur = tijd)
for i in range(n_frames - 1):
    ax.plot(marker_2[i:i+2, 0],  # X
            marker_2[i:i+2, 2],  # Z
            marker_2[i:i+2, 1],  # Y (omhoog)
            color=colors[i])

# Statische marker tekenen als grijze lijn
ax.plot(marker_1[:, 0], marker_1[:, 2], marker_1[:, 1],
        color='gray', linewidth=2, label='Statische marker')

# Aanraakmoment markeren
if aanraak_indexen.size > 0:
    i = aanraak_indexen[0]
    ax.scatter(marker_2[i, 0], marker_2[i, 2], marker_2[i, 1],
               color='red', s=60, label='Aanraakmoment')

# Aslabels en titel
ax.set_xlabel("X [mm]")
ax.set_ylabel("Z [mm]")
ax.set_zlabel("Y [mm] (omhoog)")
ax.set_title("3D traject van bewegende marker t.o.v. statische marker")

# Tijdkleurenschaal
sm = plt.cm.ScalarMappable(cmap=cm.viridis, norm=plt.Normalize(vmin=0, vmax=n_frames / framerate))
sm.set_array([])
cbar = plt.colorbar(sm, ax=ax, shrink=0.6, pad=0.1)
cbar.set_label('Tijd (seconden)')

# Y-as echt omhoog (elevation = Y)
ax.view_init(elev=20, azim=130)

ax.legend()
plt.tight_layout()
plt.show()







### Dit script leest een JSON-bestand met markerdata van een Qualisys-systeem en plot de beweging van de markers in 3D.
# import json
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Laad JSON-bestand
# with open("D:\BMT\Master\Thesis\Arduino\prothese\qualisys_data\movement_test_data.json") as f:
#     data = json.load(f)

# # Haal tijdsfrequentie op
# frequency = data["Timebase"]["Frequency"]

# # Haal alle marker-namen op uit het JSON-bestand
# marker_names = [marker["Name"] for marker in data["Markers"]]

# # Functie om markerdata op te halen
# def get_marker_data(marker_name):
#     for marker in data["Markers"]:
#         if marker["Name"] == marker_name:
#             return marker["Parts"][0]["Values"]
#     return None

# # Haal de data voor alle markers op
# marker_data = [get_marker_data(name) for name in marker_names]

# # Controleer of alle markers zijn gevonden
# if None in marker_data:
#     raise ValueError("Niet alle markers zijn gevonden in de JSON.")

# # Zet om naar arrays van x, y, z voor elke marker (verander y en z)
# timestamps = [frame_index / frequency for frame_index in range(len(marker_data[0]))]
# marker_positions = []

# # Haal de x, y, z-coördinaten van de markers
# for i in range(len(marker_data)):
#     # Hier wisselen we de y- en z-coördinaten om
#     positions = np.array([frame[0:1] + frame[2:3] + frame[1:2] for frame in marker_data[i]])  # x, z, y
#     marker_positions.append(positions)

# # Maak een 3D-plot
# fig = plt.figure(figsize=(10, 7))
# ax = fig.add_subplot(111, projection='3d')

# # Plot de posities van de markers
# for i, positions in enumerate(marker_positions):
#     ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label=marker_names[i])

# # Voeg labels en titel toe
# ax.set_xlabel('X (mm)')
# ax.set_ylabel('Z (mm)')  # De Z-as is nu de originele Y-as
# ax.set_zlabel('Y (mm)')  # De Y-as is nu de originele Z-as
# ax.set_title('Beweging van de Markers in 3D over Tijd')
# ax.legend()

# plt.show()








### dit was een poging voor een .c3d bestand, maar dat werkte niet
# import c3d
# import numpy as np
# import matplotlib.pyplot as plt

# marker_data = []

# with open('D:\BMT\Master\Thesis\Arduino\prothese\qualisys_data\movement_test_data.c3d', 'rb') as handle:
#     reader = c3d.Reader(handle)

#     for frame in reader.read_frames():
#         if isinstance(frame, tuple):
#             points = frame[0]
#         else:
#             points = frame

#         # Check of het frame geldig is
#         if isinstance(points, np.ndarray):
#             marker_data.append(points.copy())  # of np.copy(points)

# # Zet om naar numpy array
# marker_data = np.array(marker_data)

# print("Shape marker_data:", marker_data.shape)  # zou (n_frames, n_markers, 5) moeten zijn

# # Alleen verdergaan als de shape klopt
# if marker_data.ndim == 3:
#     marker_index = 0
#     x = marker_data[:, marker_index, 0]
#     y = marker_data[:, marker_index, 1]
#     z = marker_data[:, marker_index, 2]

#     plt.plot(x, label='X')
#     plt.plot(y, label='Y')
#     plt.plot(z, label='Z')
#     plt.title(f'Marker {marker_index} Positie')
#     plt.xlabel('Frame')
#     plt.ylabel('Positie (mm)')
#     plt.legend()
#     plt.grid(True)
#     plt.show()
# else:
#     print("❌ Marker data heeft geen verwachte 3D-vorm. Er ging iets mis bij het uitlezen.")
