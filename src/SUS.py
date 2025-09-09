import numpy as np

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

# Output Algoritme 1
print("Algorithm 1 SUS scores:", scores_alg1)
print(f"Algorithm 1 gemiddelde SUS score: {avg_alg1:.2f}")
print(f"Algorithm 1 standaarddeviatie: {std_alg1:.2f}")
print("Gemiddelde per vraag Alg 1:", np.round(avg_per_vraag_alg1, 2))
print("Std per vraag Alg 1:", np.round(std_per_vraag_alg1, 2))
print("SUS-verwerkte gemiddelde per vraag Alg 1:", np.round(sus_avg_per_vraag_alg1, 2))
print("SUS-verwerkte std per vraag Alg 1:", np.round(sus_std_per_vraag_alg1, 2), "\n")

# Output Algoritme 2
print("Algorithm 2 SUS scores:", scores_alg2)
print(f"Algorithm 2 gemiddelde SUS score: {avg_alg2:.2f}")
print(f"Algorithm 2 standaarddeviatie: {std_alg2:.2f}")
print("Gemiddelde per vraag Alg 2:", np.round(avg_per_vraag_alg2, 2))
print("Std per vraag Alg 2:", np.round(std_per_vraag_alg2, 2))
print("SUS-verwerkte gemiddelde per vraag Alg 2:", np.round(sus_avg_per_vraag_alg2, 2))
print("SUS-verwerkte std per vraag Alg 2:", np.round(sus_std_per_vraag_alg2, 2))

print("Gemiddelde SUS score per persoon Alg 1:", np.round(scores_alg1, 2))
print("Gemiddelde SUS score per persoon Alg 2:", np.round(scores_alg2, 2))

# import numpy as np

# def bereken_sus_score(antwoorden):
#     """
#     Bereken SUS score van een lijst van 10 antwoorden (1-5).
#     """
#     assert len(antwoorden) == 10, "Er moeten precies 10 antwoorden zijn"
#     score = 0
#     for i, antwoord in enumerate(antwoorden):
#         if (i + 1) % 2 == 1:  # oneven vragen: positief
#             score += antwoord - 1
#         else:  # even vragen: negatief
#             score += 5 - antwoord
#     sus_score = score * 2.5
#     return sus_score

# def analyseer_sus_scores(alle_antwoorden):
#     """
#     alle_antwoorden: lijst van lijsten, elke sublijst is antwoorden van 10 vragen per deelnemer.
#     Geeft lijst met SUS scores, gemiddelde, std, plus gemiddeldes en std per vraag.
#     """
#     sus_scores = [bereken_sus_score(antwoorden) for antwoorden in alle_antwoorden]
#     gemiddelde = np.mean(sus_scores)
#     std_dev = np.std(sus_scores, ddof=1)  # sample std dev

#     # Antwoorden per vraag (kolommen)
#     antwoorden_per_vraag = np.array(alle_antwoorden).T  # shape (10 vragen, aantal deelnemers)
#     gemiddelden_per_vraag = np.mean(antwoorden_per_vraag, axis=1)
#     stds_per_vraag = np.std(antwoorden_per_vraag, axis=1, ddof=1)

#     return sus_scores, gemiddelde, std_dev, gemiddelden_per_vraag, stds_per_vraag

# # Vul hier de antwoorden per deelnemer in voor Algoritme 1 en Algoritme 2:
# sus_alg1 = [
#     [2, 1, 5, 3, 5, 1, 5, 1, 4, 2],
#     [3, 2, 4, 3, 4, 2, 4, 2, 4, 1],
#     [4, 1, 4, 2, 4, 1, 5, 2, 4, 1],
#     [3, 1, 4, 3, 3, 2, 5, 1, 4, 1],
#     [2, 4, 4, 1, 5, 1, 5, 4, 5, 1],
#     [4, 1, 5, 2, 5, 1, 5, 3, 5, 1],
#     [3, 1, 5, 2, 5, 1, 5, 1, 5, 1],
#     [2, 2, 5, 1, 4, 1, 5, 2, 4, 1],
#     [4, 2, 4, 2, 4, 2, 4, 3, 4, 2],
#     [4, 1, 3, 1, 4, 2, 4, 2, 3, 1]
# ]

# sus_alg2 = [
#     [1, 4, 1, 3, 3, 2, 1, 5, 2, 1],
#     [3, 2, 4, 3, 4, 3, 3, 2, 4, 2],
#     [4, 1, 5, 1, 5, 1, 5, 1, 5, 1],
#     [3, 3, 2, 3, 3, 2, 4, 2, 2, 3],
#     [5, 1, 5, 1, 4, 1, 5, 1, 5, 1],
#     [2, 1, 4, 4, 4, 3, 5, 2, 4, 2],
#     [3, 1, 5, 1, 4, 1, 5, 2, 4, 1],
#     [4, 1, 4, 3, 3, 1, 4, 1, 4, 2],
#     [3, 2, 3, 2, 4, 5, 4, 3, 4, 3],
#     [3, 2, 4, 1, 4, 1, 4, 2, 3, 2]
# ]

# scores_alg1, avg_alg1, std_alg1, avg_per_vraag_alg1, std_per_vraag_alg1 = analyseer_sus_scores(sus_alg1)
# scores_alg2, avg_alg2, std_alg2, avg_per_vraag_alg2, std_per_vraag_alg2 = analyseer_sus_scores(sus_alg2)

# print("Algorithm 1 SUS scores:", scores_alg1)
# print(f"Algorithm 1 gemiddelde SUS score: {avg_alg1:.2f}")
# print(f"Algorithm 1 standaarddeviatie: {std_alg1:.2f}")
# print("Gemiddelde per vraag Alg 1:", np.round(avg_per_vraag_alg1, 2))
# print("Std per vraag Alg 1:", np.round(std_per_vraag_alg1, 2), "\n")

# print("Algorithm 2 SUS scores:", scores_alg2)
# print(f"Algorithm 2 gemiddelde SUS score: {avg_alg2:.2f}")
# print(f"Algorithm 2 standaarddeviatie: {std_alg2:.2f}")
# print("Gemiddelde per vraag Alg 2:", np.round(avg_per_vraag_alg2, 2))
# print("Std per vraag Alg 2:", np.round(std_per_vraag_alg2, 2))





# sus_alg1 = [
#     [2, 1, 5, 3, 5, 1, 5, 1, 4, 2],
#     [3, 2, 4, 3, 4, 2, 4, 2, 4, 1],
#     [4, 1, 4, 2, 4, 1, 5, 2, 4, 1],
#     [3, 1, 4, 3, 3, 2, 5, 1, 4, 1],
#     [2, 4, 4, 1, 5, 1, 5, 4, 5, 1],
#     [4, 1, 5, 2, 5, 1, 5, 3, 5, 1],
#     [3, 1, 5, 2, 5, 1, 5, 1, 5, 1],
#     [2, 2, 5, 1, 4, 1, 5, 2, 4, 1],
#     [4, 2, 4, 2, 4, 2, 4, 3, 4, 2],
#     [4, 1, 3, 1, 4, 2, 4, 2, 3, 1]
# ]

# sus_alg2 = [
#     [1, 4, 1, 3, 3, 2, 1, 5, 2, 1],
#     [3, 2, 4, 3, 4, 3, 3, 2, 4, 2],
#     [4, 1, 5, 1, 5, 1, 5, 1, 5, 1],
#     [3, 3, 2, 3, 3, 2, 4, 2, 2, 3],
#     [5, 1, 5, 1, 4, 1, 5, 1, 5, 1],
#     [2, 1, 4, 4, 4, 3, 5, 2, 4, 2],
#     [3, 1, 5, 1, 4, 1, 5, 2, 4, 1],
#     [4, 1, 4, 3, 3, 1, 4, 1, 4, 2],
#     [3, 2, 3, 2, 4, 5, 4, 3, 4, 3],
#     [3, 2, 4, 1, 4, 1, 4, 2, 3, 2]
# ]