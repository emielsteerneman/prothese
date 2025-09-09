import pandas as pd
import scipy.stats as stats
import statsmodels.api as sm
import matplotlib.pyplot as plt

# Gegevens invoeren
data = pd.DataFrame({
    'Participant': ['Anna', 'Chris', 'Lieke', 'Mart', 'Thijs', 'Bas', 'Corentin', 'Tom', 'Dylan', 'Wouter'],
    'MAE_1': [1.5512, 1.5447, 1.5212, 1.1033, 1.4056, 1.5191, 1.3418, 0.9397, 1.2006, 1.1340],
    'SD_1': [0.4409, 0.2965, 0.7432, 0.2728, 0.4304, 0.2158, 0.4370, 0.0882, 0.3923, 0.1423],
    'MAE_2': [1.8705, 2.4970, 1.6735, 1.7092, 1.8589, 2.7941, 1.8581, 1.6801, 2.5374, 1.9506],
    'SD_2': [0.2633, 0.4519, 0.2711, 0.2756, 0.3757, 0.5878, 0.3531, 0.3351, 0.5011, 0.3257]
})

# Verschillen berekenen
data['MAE_diff'] = data['MAE_2'] - data['MAE_1']
data['SD_diff'] = data['SD_2'] - data['SD_1']

# QQ-plots
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
sm.qqplot(data['MAE_diff'], line='s', ax=axes[0])
axes[0].set_title('QQ-plot MAE verschil')

sm.qqplot(data['SD_diff'], line='s', ax=axes[1])
axes[1].set_title('QQ-plot SD verschil')

plt.tight_layout()
plt.show()

# Shapiro-Wilk normaliteitstest
mae_sw_stat, mae_sw_p = stats.shapiro(data['MAE_diff'])
sd_sw_stat, sd_sw_p = stats.shapiro(data['SD_diff'])

print("\n📏 Shapiro-Wilk Test op normaliteit van verschillen:")
print(f"MAE verschil: p = {mae_sw_p:.4f} → {'✅ Normaal verdeeld' if mae_sw_p > 0.05 else '❌ Niet normaal verdeeld'}")
print(f"SD verschil:  p = {sd_sw_p:.4f} → {'✅ Normaal verdeeld' if sd_sw_p > 0.05 else '❌ Niet normaal verdeeld'}")

# Kies test op basis van normaliteit
if mae_sw_p > 0.05:
    mae_stat, mae_p = stats.ttest_rel(data['MAE_1'], data['MAE_2'])
    print(f"\n🔎 MAE Paired t-test: t = {mae_stat:.3f}, p = {mae_p:.4f}")
else:
    mae_stat, mae_p = stats.wilcoxon(data['MAE_1'], data['MAE_2'])
    print(f"\n🔎 MAE Wilcoxon-test: stat = {mae_stat:.3f}, p = {mae_p:.4f}")

if sd_sw_p > 0.05:
    sd_stat, sd_p = stats.ttest_rel(data['SD_1'], data['SD_2'])
    print(f"🔎 SD Paired t-test: t = {sd_stat:.3f}, p = {sd_p:.4f}")
else:
    sd_stat, sd_p = stats.wilcoxon(data['SD_1'], data['SD_2'])
    print(f"🔎 SD Wilcoxon-test: stat = {sd_stat:.3f}, p = {sd_p:.4f}")

corr, p_val = stats.pearsonr(data['MAE_1'], data['MAE_2'])
print(f"Pearson correlatie MAE tussen Alg1 en Alg2: r = {corr:.3f}, p = {p_val:.4f}")


import scipy.stats as stats

# Aannames:
# De eerste 5 proefpersonen (index 0 t/m 4) hebben volgorde 1→2
# De laatste 5 proefpersonen (index 5 t/m 9) hebben volgorde 2→1

# Maak verschil-scores MAE
data['MAE_diff'] = data['MAE_2'] - data['MAE_1']

# Splits de data op basis van index
group1 = data.loc[0:4, 'MAE_diff']  # volgorde 1→2
group2 = data.loc[5:9, 'MAE_diff']  # volgorde 2→1

# Optioneel: check normaliteit per groep
for i, group in enumerate([group1, group2], start=1):
    stat, p = stats.shapiro(group)
    print(f'Normaliteit MAE_diff in groep {i}: p = {p:.4f}')

# Kies test op basis van normaliteit (hier assumeer t-test)
t_stat, p_val = stats.ttest_ind(group1, group2)
print(f'\nInvloed volgorde op MAE verschil-score (onafhankelijke t-test): t={t_stat:.3f}, p={p_val:.4f}')


import scipy.stats as stats

# Eerste 5 deelnemers: volgorde Alg1 → Alg2
group1 = data.loc[0:4]
corr1, p1 = stats.pearsonr(group1['MAE_1'], group1['MAE_2'])
print(f"Groep 1 (Alg1 eerst) - Pearson correlatie MAE: r = {corr1:.3f}, p = {p1:.4f}")

# Laatste 5 deelnemers: volgorde Alg2 → Alg1
group2 = data.loc[5:9]
corr2, p2 = stats.pearsonr(group2['MAE_1'], group2['MAE_2'])
print(f"Groep 2 (Alg2 eerst) - Pearson correlatie MAE: r = {corr2:.3f}, p = {p2:.4f}")


import matplotlib.pyplot as plt
import numpy as np

# Data splitsen
group1 = data.loc[0:4]
group2 = data.loc[5:9]

plt.figure(figsize=(10,5))

# Plot groep 1
plt.subplot(1, 2, 1)
plt.scatter(group1['MAE_1'], group1['MAE_2'], color='blue')
m, b = np.polyfit(group1['MAE_1'], group1['MAE_2'], 1)
plt.plot(group1['MAE_1'], m*group1['MAE_1'] + b, color='blue')
plt.title('Groep 1: Alg1 eerst')
plt.xlabel('MAE Algoritme 1')
plt.ylabel('MAE Algoritme 2')
plt.grid(True)

# Plot groep 2
plt.subplot(1, 2, 2)
plt.scatter(group2['MAE_1'], group2['MAE_2'], color='red')
m, b = np.polyfit(group2['MAE_1'], group2['MAE_2'], 1)
plt.plot(group2['MAE_1'], m*group2['MAE_1'] + b, color='red')
plt.title('Groep 2: Alg2 eerst')
plt.xlabel('MAE Algoritme 1')
plt.ylabel('MAE Algoritme 2')
plt.grid(True)

plt.tight_layout()
plt.show()
