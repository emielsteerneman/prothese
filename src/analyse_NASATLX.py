import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import shapiro, ttest_rel, wilcoxon

# Jouw originele data
alg1 = np.array([
    [11, 10, 10, 5, 14, 1],
    [5, 14, 3, 16, 9, 1],
    [2, 2, 5, 3, 3, 3],
    [2, 5, 1, 3, 2, 1],
    [8, 13, 1, 4, 6, 3],
    [2, 4, 2, 5, 3, 2],
    [3, 6, 4, 4, 3, 1],
    [15, 14, 2, 3, 4, 3],
    [3, 5, 3, 17, 8, 5],
    [9, 7, 6, 17, 7, 10],
])
alg2 = np.array([
    [15, 11, 10, 15, 17, 2],
    [8, 12, 3, 15, 4, 1],
    [3, 11, 8, 6, 13, 4],
    [3, 1, 1, 3, 1, 1],
    [3, 3, 1, 1, 3, 1],
    [3, 5, 5, 4, 7, 5],
    [4, 7, 8, 5, 5, 1],
    [12, 7, 2, 6, 13, 4],
    [5, 8, 4, 14, 5, 8],
    [12, 8, 5, 16, 8, 7],
])

labels = ['Mental Demand', 'Physical Demand', 'Temporal Demand', 'Performance', 'Effort', 'Frustration']

print("📈 Shapiro-Wilk normality test en gepaste test voor verschil tussen Algoritme 1 en 2:")

for i, label in enumerate(labels):
    # Check normaliteit van verschillen (normale aanname van paired t-test is dat verschil normaal verdeeld is)
    diffs = alg1[:, i] - alg2[:, i]
    stat_diff, p_diff = shapiro(diffs)

    if p_diff > 0.05:
        # normaal → paired t-test
        t_stat, p_val = ttest_rel(alg1[:, i], alg2[:, i])
        test_name = "Paired t-test"
    else:
        # niet normaal → Wilcoxon signed-rank test
        t_stat, p_val = wilcoxon(alg1[:, i], alg2[:, i])
        test_name = "Wilcoxon signed-rank test"

    print(f"{label}: {test_name} → stat = {t_stat:.3f}, p = {p_val:.4f}")


from scipy.stats import ttest_ind, mannwhitneyu

# Groepsindeling op basis van volgorde
# Eerst 5: volgorde 1 (Alg1 eerst), tweede 5: volgorde 2 (Alg2 eerst)
group_order = np.array([1]*5 + [2]*5)

print("\n🔄 Volgorde-effect (tussen groepen) per dimensie:")

for i, label in enumerate(labels):
    # Scores van Alg1 en Alg2 per volgorde-groep
    alg1_g1 = alg1[group_order == 1, i]
    alg1_g2 = alg1[group_order == 2, i]
    alg2_g1 = alg2[group_order == 1, i]
    alg2_g2 = alg2[group_order == 2, i]

    # Check normaliteit per groep voor Alg1
    p_alg1_g1 = shapiro(alg1_g1).pvalue
    p_alg1_g2 = shapiro(alg1_g2).pvalue
    # Check normaliteit per groep voor Alg2
    p_alg2_g1 = shapiro(alg2_g1).pvalue
    p_alg2_g2 = shapiro(alg2_g2).pvalue

    # Kies test voor Alg1 volgorde effect
    if p_alg1_g1 > 0.05 and p_alg1_g2 > 0.05:
        test_alg1 = "t-test"
        stat_alg1, pval_alg1 = ttest_ind(alg1_g1, alg1_g2)
    else:
        test_alg1 = "Mann-Whitney U"
        stat_alg1, pval_alg1 = mannwhitneyu(alg1_g1, alg1_g2, alternative='two-sided')

    # Kies test voor Alg2 volgorde effect
    if p_alg2_g1 > 0.05 and p_alg2_g2 > 0.05:
        test_alg2 = "t-test"
        stat_alg2, pval_alg2 = ttest_ind(alg2_g1, alg2_g2)
    else:
        test_alg2 = "Mann-Whitney U"
        stat_alg2, pval_alg2 = mannwhitneyu(alg2_g1, alg2_g2, alternative='two-sided')

    print(f"{label}:")
    print(f"  Alg1 volgorde effect: {test_alg1} stat={stat_alg1:.3f}, p={pval_alg1:.4f}")
    print(f"  Alg2 volgorde effect: {test_alg2} stat={stat_alg2:.3f}, p={pval_alg2:.4f}")
