import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
import scipy.signal as signal

# Bestandspad aanpassen
bestandspad = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250312_162604.txt"

x_values = []
y_values = []

# Bestand inlezen en verwerken
with open(bestandspad, "r") as f:
    for line in f:
        links, *rechts = line.strip().split()  # Splits op spaties
        rechts = " ".join(rechts)
        # if len(delen) == 2:  # Verwacht formaat controleren
        # _, data = delen
        getallen = rechts.split(",")  # Splits de laatste twee getallen
        if len(getallen) >= 3:  # Controleer of er genoeg data is
            try:
                x = float(getallen[0]) 
                y = float(getallen[2])  
                
                x_values.append(x)  # ✅ Append values inside the loop
                y_values.append(y)

            except ValueError:
                print(f"Fout bij verwerken van regel: {line}")


# y_values = list(np.diff(y_values))
# y_values = [0] + y_values

if len(y_values) > 10:  # ✅ Ensure there's enough data for filtering
    fs = 1000  # Sample rate in Hz (adjust if needed)
    f0 = 50    # Frequency to remove (50Hz noise)
    Q = 30     # Quality factor (adjust for sharpness of notch)

    # Create notch filter coefficients
    b, a = signal.iirnotch(f0, Q, fs)

    # Apply notch filter to y-values
    filtered_y_values = signal.filtfilt(b, a, y_values)
    
    # 📈 **Plot Original vs Filtered Data**
    plt.figure(figsize=(10, 5))
    plt.plot(x_values, y_values, marker="o", linestyle="-", label="Original Data", alpha=0.5)
    # plt.plot(x_values, filtered_y_values, marker="o", linestyle="-", label="Filtered (50Hz Notch)", color="red")
    plt.xlabel("Timestamp")
    plt.ylabel("Input")
    plt.title("Encoder Angular Velocity (degrees/s)")
    plt.legend()
    plt.grid(True)
    plt.show()
else:
    print("Niet genoeg data om filtering toe te passen.")














# import matplotlib.pyplot as plt
# from collections import defaultdict

# # Bestandspad aanpassen
# bestandspad = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250303_10300tot11800_motorspeed_encoderspeed.txt"

# # Dictionary om x-waarden te groeperen en bijbehorende y-waarden op te slaan
# data_dict = defaultdict(list)

# # Bestand inlezen en verwerken
# with open(bestandspad, "r") as f:
#     for line in f:
#         delen = line.strip().split()  # Splits op spaties
#         if len(delen) == 2:  # Verwacht formaat controleren
#             _, data = delen
#             getallen = data.split(",")  # Splits de laatste twee getallen
#             if len(getallen) >= 2:  # Controleer of er genoeg data is
#                 try:
#                     x = float(getallen[-3])  # Twee-na-laatste getal (X-as)
#                     y = abs(float(getallen[-1]))  # Absolute waarde van het laatste getal (Y-as)
#                     data_dict[x].append(y)  # Opslaan in dictionary
#                 except ValueError:
#                     print(f"Fout bij verwerken van regel: {line}")

# # Nieuwe x- en y-waarden berekenen
# x_values = []
# y_values = []

# for x, y_list in data_dict.items():
#     if len(y_list) > 0:  # Voorkom deling door nul
#         gemiddelde_y = (sum(y_list) / len(y_list)) # Gemiddelde berekenen en door 2 delen
#         x_values.append(x)
#         y_values.append(y_list)

# # Sorteer de waarden op x-as voor een doorlopende lijn
# # sorted_pairs = sorted(zip(x_values, y_values))
# # x_values, y_values = zip(*sorted_pairs)  # Splits de gesorteerde paren

# # Data plotten als lijn
# plt.plot(x_values, y_values, marker="o", linestyle="-")  # Doorlopende lijn met punten
# plt.xlabel("X-waarde (een-na-laatste getal)")
# plt.ylabel("Gemiddelde van absolute Y-waarden gedeeld door 2")
# plt.title("Lijnplot met verwerkte gegevens")
# plt.grid(True)
# plt.show()
