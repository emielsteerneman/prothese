import numpy as np
from scipy.stats import shapiro, ttest_rel, wilcoxon

def bereken_sus_score(antwoorden):
    """
    Bereken SUS score van een lijst van 10 antwoorden (1-5).
    """
    assert len(antwoorden) == 10, "Er moeten precies 10 antwoorden zijn"
    score = 0
    for i, antwoord in enumerate(antwoorden):
        if (i + 1) % 2 == 1:  # oneven vragen: positief
            score += antwoord - 1
        else:  # even vragen: negatief
            score += 5 - antwoord
    sus_score = score * 2.5
    return sus_score

def analyseer_sus_scores(alle_antwoorden):
    """
    alle_antwoorden: lijst van lijsten, elke sublijst is antwoorden van 10 vragen per deelnemer.
    Geeft lijst met SUS scores, gemiddelde, std, plus gemiddeldes en std per vraag (ruw en verwerkt).
    """
    sus_scores = [bereken_sus_score(antwoorden) for antwoorden in alle_antwoorden]
    gemiddelde = np.mean(sus_scores)
    std_dev = np.std(sus_scores, ddof=1)  # sample std dev

    # Antwoorden per vraag (kolommen)
    antwoorden_per_vraag = np.array(alle_antwoorden).T  # shape (10 vragen, aantal deelnemers)

    # Ruwe gemiddelden en stds per vraag
    gemiddelden_per_vraag = np.mean(antwoorden_per_vraag, axis=1)
    stds_per_vraag = np.std(antwoorden_per_vraag, axis=1, ddof=1)

    # SUS-verwerkte scores per vraag
    verwerkte_antwoorden = []
    for i in range(10):
        if (i + 1) % 2 == 1:  # oneven vragen
            verwerkte = antwoorden_per_vraag[i] - 1
        else:  # even vragen
            verwerkte = 5 - antwoorden_per_vraag[i]
        verwerkte_antwoorden.append(verwerkte)
    verwerkte_antwoorden = np.array(verwerkte_antwoorden)

    sus_gemiddelden_per_vraag = np.mean(verwerkte_antwoorden, axis=1)
    sus_stds_per_vraag = np.std(verwerkte_antwoorden, axis=1, ddof=1)

    return (
        sus_scores, gemiddelde, std_dev,
        gemiddelden_per_vraag, stds_per_vraag,
        sus_gemiddelden_per_vraag, sus_stds_per_vraag
    )

# Antwoorden van deelnemers voor Algoritme 1
sus_alg1 = [
    [2, 1, 5, 3, 5, 1, 5, 1, 4, 2],
    [3, 2, 4, 3, 4, 2, 4, 2, 4, 1],
    [4, 1, 4, 2, 4, 1, 5, 2, 4, 1],
    [3, 1, 4, 3, 3, 2, 5, 1, 4, 1],
    [2, 4, 4, 1, 5, 1, 5, 4, 5, 1],
    [4, 1, 5, 2, 5, 1, 5, 3, 5, 1],
    [3, 1, 5, 2, 5, 1, 5, 1, 5, 1],
    [2, 2, 5, 1, 4, 1, 5, 2, 4, 1],
    [4, 2, 4, 2, 4, 2, 4, 3, 4, 2],
    [4, 1, 3, 1, 4, 2, 4, 2, 3, 1]
]

# Antwoorden van deelnemers voor Algoritme 2
sus_alg2 = [
    [1, 4, 1, 3, 3, 2, 1, 5, 2, 1],
    [3, 2, 4, 3, 4, 3, 3, 2, 4, 2],
    [4, 1, 5, 1, 5, 1, 5, 1, 5, 1],
    [3, 3, 2, 3, 3, 2, 4, 2, 2, 3],
    [5, 1, 5, 1, 4, 1, 5, 1, 5, 1],
    [2, 1, 4, 4, 4, 3, 5, 2, 4, 2],
    [3, 1, 5, 1, 4, 1, 5, 2, 4, 1],
    [4, 1, 4, 3, 3, 1, 4, 1, 4, 2],
    [3, 2, 3, 2, 4, 5, 4, 3, 4, 3],
    [3, 2, 4, 1, 4, 1, 4, 2, 3, 2]
]

# Analyse
(
    scores_alg1, avg_alg1, std_alg1,
    avg_per_vraag_alg1, std_per_vraag_alg1,
    sus_avg_per_vraag_alg1, sus_std_per_vraag_alg1
) = analyseer_sus_scores(sus_alg1)

(
    scores_alg2, avg_alg2, std_alg2,
    avg_per_vraag_alg2, std_per_vraag_alg2,
    sus_avg_per_vraag_alg2, sus_std_per_vraag_alg2
) = analyseer_sus_scores(sus_alg2)

def paired_test(x, y, naam):
    print(f"\nAnalyse voor {naam}:")
    stat1, p1 = shapiro(x)
    stat2, p2 = shapiro(y)
    print(f"  Shapiro-Wilk p-waarde Alg1: {p1:.4f} -> {'Normaal' if p1 > 0.05 else 'Niet normaal'}")
    print(f"  Shapiro-Wilk p-waarde Alg2: {p2:.4f} -> {'Normaal' if p2 > 0.05 else 'Niet normaal'}")
    
    if p1 > 0.05 and p2 > 0.05:
        stat, p = ttest_rel(x, y)
        print(f"  Paired t-test: stat={stat:.3f}, p={p:.4f}")
    else:
        stat, p = wilcoxon(x, y)
        print(f"  Wilcoxon signed-rank test: stat={stat:.3f}, p={p:.4f}")

# Test op totale SUS score
paired_test(scores_alg1, scores_alg2, "totale SUS score")

# Test per vraag (ruwe antwoorden)
for i in range(10):
    vraag_alg1 = np.array(sus_alg1)[:, i]
    vraag_alg2 = np.array(sus_alg2)[:, i]
    paired_test(vraag_alg1, vraag_alg2, f"vraag {i+1}")

# Samenvatting
print(f"\nGemiddelde SUS score Alg1: {avg_alg1:.2f} (std {std_alg1:.2f})")
print(f"Gemiddelde SUS score Alg2: {avg_alg2:.2f} (std {std_alg2:.2f})")




from scipy.stats import shapiro, ttest_ind, mannwhitneyu

# Aantal deelnemers (hier 10)
n = len(scores_alg1)

# Volgorde afgeleid uit index: eerste 5 -> volgorde 1, volgende 5 -> volgorde 2
volgorde = [1]*5 + [2]*5

def analyseer_volgorde_effect(scores, volgorde, naam_alg):
    groep1_scores = [score for score, ord in zip(scores, volgorde) if ord == 1]
    groep2_scores = [score for score, ord in zip(scores, volgorde) if ord == 2]

    print(f"\nVolgorde-effect voor {naam_alg}:")
    stat1, p1 = shapiro(groep1_scores)
    stat2, p2 = shapiro(groep2_scores)
    print(f"  Shapiro-Wilk p-waarde groep 1: {p1:.4f} {'(normaal)' if p1>0.05 else '(niet normaal)'}")
    print(f"  Shapiro-Wilk p-waarde groep 2: {p2:.4f} {'(normaal)' if p2>0.05 else '(niet normaal)'}")

    if p1 > 0.05 and p2 > 0.05:
        stat, p = ttest_ind(groep1_scores, groep2_scores, equal_var=False)
        testnaam = "Onafhankelijke t-test"
    else:
        stat, p = mannwhitneyu(groep1_scores, groep2_scores, alternative='two-sided')
        testnaam = "Mann-Whitney U-test"

    print(f"  {testnaam}: stat={stat:.3f}, p={p:.4f}")

# Voer analyse uit
analyseer_volgorde_effect(scores_alg1, volgorde, "Algoritme 1 SUS score")
analyseer_volgorde_effect(scores_alg2, volgorde, "Algoritme 2 SUS score")

from scipy.stats import ttest_ind, mannwhitneyu, shapiro

def volgorde_effect_per_vraag(sus_data_alg, label_alg):
    print(f"Volgorde-effect per vraag voor {label_alg}:")
    # Verdeel deelnemers op volgorde: eerste 5 doen Alg eerst, tweede 5 doen Alg 2 eerst
    groep1 = np.array(sus_data_alg[:5])  # deelnemers volgorde 1
    groep2 = np.array(sus_data_alg[5:])  # deelnemers volgorde 2

    for vraag_idx in range(10):
        scores_groep1 = groep1[:, vraag_idx]
        scores_groep2 = groep2[:, vraag_idx]

        # Normaliteit testen
        p_groep1 = shapiro(scores_groep1).pvalue
        p_groep2 = shapiro(scores_groep2).pvalue

        # Kies test op basis van normaliteit
        if p_groep1 > 0.05 and p_groep2 > 0.05:
            stat, p = ttest_ind(scores_groep1, scores_groep2)
            testnaam = "Onafhankelijke t-test"
        else:
            stat, p = mannwhitneyu(scores_groep1, scores_groep2)
            testnaam = "Mann-Whitney U test"

        print(f"Vraag {vraag_idx + 1}:")
        print(f"  Shapiro-Wilk p-waarde groep 1: {p_groep1:.4f} ({'normaal' if p_groep1>0.05 else 'niet normaal'})")
        print(f"  Shapiro-Wilk p-waarde groep 2: {p_groep2:.4f} ({'normaal' if p_groep2>0.05 else 'niet normaal'})")
        print(f"  {testnaam}: stat={stat:.3f}, p={p:.4f}\n")

# Pas dit aan met je originele data variabelen voor de SUS antwoorden (raw antwoorden, niet de scores!)
volgorde_effect_per_vraag(sus_alg1, "Algoritme 1")
volgorde_effect_per_vraag(sus_alg2, "Algoritme 2")
