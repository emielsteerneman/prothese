import matplotlib.pyplot as plt
from collections import defaultdict

# Bestandspad aanpassen
bestandspad = "D:\BMT\Master\Thesis\Arduino\prothese\logs\log_20250305_test2.txt"

# Dictionary om x-waarden te groeperen en bijbehorende y-waarden op te slaan
data_dict = defaultdict(list)

# Bestand inlezen en verwerken
with open(bestandspad, "r") as f:
    for line in f:
        delen = line.strip().split()  # Splits op spaties
        if len(delen) == 2:  # Verwacht formaat controleren
            _, data = delen
            getallen = data.split(",")  # Splits de laatste twee getallen
            if len(getallen) >= 2:  # Controleer of er genoeg data is
                try:
                    x = float(getallen[-3])  # Twee-na-laatste getal (X-as)
                    y = abs(float(getallen[-1]))  # Absolute waarde van het laatste getal (Y-as)
                    data_dict[x].append(y)  # Opslaan in dictionary
                except ValueError:
                    print(f"Fout bij verwerken van regel: {line}")

# Nieuwe x- en y-waarden berekenen
x_values = []
y_values = []

for x, y_list in data_dict.items():
    if len(y_list) > 0:  # Voorkom deling door nul
        gemiddelde_y = (sum(y_list) / len(y_list)) # Gemiddelde berekenen en door 2 delen
        x_values.append(x)
        y_values.append(gemiddelde_y)

# Sorteer de waarden op x-as voor een doorlopende lijn
sorted_pairs = sorted(zip(x_values, y_values))
x_values, y_values = zip(*sorted_pairs)  # Splits de gesorteerde paren

# Data plotten als lijn
plt.plot(x_values, y_values, marker="o", linestyle="-")  # Doorlopende lijn met punten
plt.xlabel("X-waarde (een-na-laatste getal)")
plt.ylabel("Gemiddelde van absolute Y-waarden gedeeld door 2")
plt.title("Lijnplot met verwerkte gegevens")
plt.grid(True)
plt.show()
