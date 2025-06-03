
alg1_dim1 = [11, 5, 2, 2, 8, 2, 3, 15, 3, 9]
alg1_dim2 = [10, 14, 2, 5, 13, 4, 6, 14, 5, 7]
alg1_dim3 = [10, 3, 5, 1, 1, 2, 4, 2, 3, 6]
alg1_dim4 = [5, 16, 3, 3, 4, 5, 4, 3, 17, 17]
alg1_dim5 = [14, 9, 3, 2, 6, 3, 3, 4, 8, 7]
alg1_dim6 = [1, 1, 3, 1, 3, 2, 1, 3, 5, 10]

alg2_dim1 = [15, 8, 3, 3, 3, 3, 4, 12, 5, 12]
alg2_dim2 = [11, 12, 11, 1, 3, 5, 7, 7, 8, 8]
alg2_dim3 = [10, 3, 8, 1, 1, 5, 8, 2, 4, 5]
alg2_dim4 = [15, 15, 6, 3, 1, 4, 5, 6, 14, 16]
alg2_dim5 = [17, 4, 13, 1, 3, 7, 5, 13, 5, 8]
alg2_dim6 = [2, 1, 4, 1, 1, 5, 1, 4, 8, 7]

# determine the average of each dimension for each algorithm
alg1_dim1_avg = sum(alg1_dim1) / len(alg1_dim1)
alg1_dim2_avg = sum(alg1_dim2) / len(alg1_dim2)
alg1_dim3_avg = sum(alg1_dim3) / len(alg1_dim3)
alg1_dim4_avg = sum(alg1_dim4) / len(alg1_dim4)
alg1_dim5_avg = sum(alg1_dim5) / len(alg1_dim5)
alg1_dim6_avg = sum(alg1_dim6) / len(alg1_dim6)

alg2_dim1_avg = sum(alg2_dim1) / len(alg2_dim1)
alg2_dim2_avg = sum(alg2_dim2) / len(alg2_dim2)
alg2_dim3_avg = sum(alg2_dim3) / len(alg2_dim3)
alg2_dim4_avg = sum(alg2_dim4) / len(alg2_dim4)
alg2_dim5_avg = sum(alg2_dim5) / len(alg2_dim5)
alg2_dim6_avg = sum(alg2_dim6) / len(alg2_dim6)

# determine the standard deviation of each dimension for each algorithm
alg1_dim1_std = (sum([(x - alg1_dim1_avg) ** 2 for x in alg1_dim1]) / len(alg1_dim1)) ** 0.5
alg1_dim2_std = (sum([(x - alg1_dim2_avg) ** 2 for x in alg1_dim2]) / len(alg1_dim2)) ** 0.5
alg1_dim3_std = (sum([(x - alg1_dim3_avg) ** 2 for x in alg1_dim3]) / len(alg1_dim3)) ** 0.5
alg1_dim4_std = (sum([(x - alg1_dim4_avg) ** 2 for x in alg1_dim4]) / len(alg1_dim4)) ** 0.5
alg1_dim5_std = (sum([(x - alg1_dim5_avg) ** 2 for x in alg1_dim5]) / len(alg1_dim5)) ** 0.5
alg1_dim6_std = (sum([(x - alg1_dim6_avg) ** 2 for x in alg1_dim6]) / len(alg1_dim6)) ** 0.5

alg2_dim1_std = (sum([(x - alg2_dim1_avg) ** 2 for x in alg2_dim1]) / len(alg2_dim1)) ** 0.5
alg2_dim2_std = (sum([(x - alg2_dim2_avg) ** 2 for x in alg2_dim2]) / len(alg2_dim2)) ** 0.5
alg2_dim3_std = (sum([(x - alg2_dim3_avg) ** 2 for x in alg2_dim3]) / len(alg2_dim3)) ** 0.5
alg2_dim4_std = (sum([(x - alg2_dim4_avg) ** 2 for x in alg2_dim4]) / len(alg2_dim4)) ** 0.5
alg2_dim5_std = (sum([(x - alg2_dim5_avg) ** 2 for x in alg2_dim5]) / len(alg2_dim5)) ** 0.5
alg2_dim6_std = (sum([(x - alg2_dim6_avg) ** 2 for x in alg2_dim6]) / len(alg2_dim6)) ** 0.5


import matplotlib.pyplot as plt
import numpy as np

# Data van Algorithm 1
labels = ['Mental Demand', 'Physical Demand', 'Temporal Demand', 'Performance', 'Effort', 'Frustration']
alg1_averages = [alg1_dim1_avg, alg1_dim2_avg, alg1_dim3_avg, alg1_dim4_avg, alg1_dim5_avg, alg1_dim6_avg]
alg1_stds = [alg1_dim1_std, alg1_dim2_std, alg1_dim3_std, alg1_dim4_std, alg1_dim5_std, alg1_dim6_std]

# Data van Algorithm 2
alg2_averages = [alg2_dim1_avg, alg2_dim2_avg, alg2_dim3_avg, alg2_dim4_avg, alg2_dim5_avg, alg2_dim6_avg]
alg2_stds = [alg2_dim1_std, alg2_dim2_std, alg2_dim3_std, alg2_dim4_std, alg2_dim5_std, alg2_dim6_std]

labels = labels[::-1]
alg1_averages = alg1_averages[::-1]
alg1_stds = alg1_stds[::-1]
alg2_averages = alg2_averages[::-1]
alg2_stds = alg2_stds[::-1]

n = len(labels)
y = np.arange(n)
bar_height = 0.35  # hoogte van één bar
delta = 0.02  # kleine extra ruimte


plt.figure(figsize=(12, 7))
plt.title("Mean NASA TLX Score ± SD - Algorithms 1 & 2", fontsize=20)
plt.xlabel("NASA TLX Score", fontsize=18)
plt.ylabel("Dimensions", fontsize=18)

# Maak y-as leeg (we tonen labels binnen de grafiek)
plt.yticks([], fontsize = 16)

plt.xticks(np.arange(0, 21, 2), fontsize=16)

# Bereken ruimte voor de bars (we plaatsen 2 bars per dimensie, iets uit elkaar)
plt.xlim(0, max(max(alg1_averages[i] + alg1_stds[i], alg2_averages[i] + alg2_stds[i]) for i in range(n)) + 5)

# Bars van Algoritme 1 iets omhoog (+bar_height/2)
plt.barh(y + bar_height/2 + delta, alg1_averages, height=bar_height, xerr=alg1_stds, capsize=5, label='Algorithm 1')

# Bars van Algoritme 2 iets omlaag (-bar_height/2)
plt.barh(y - bar_height/2 - delta, alg2_averages, height=bar_height, xerr=alg2_stds, capsize=5, color='green', label='Algorithm 2')

# Labels net rechts van de foutbalken van Algorithm 2, rechts uitgelijnd
for i in range(n):
    x_pos = alg1_averages[i] + alg1_stds[i] + 0.3
    plt.text(x_pos, y[i] + bar_height/2 + delta, labels[i], va='center', ha='left', fontsize=16)

plt.legend(fontsize=14)
plt.tight_layout()
plt.show()
