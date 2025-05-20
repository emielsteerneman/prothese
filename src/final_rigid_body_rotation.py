### hier lijkt nog van alles mis te gaan ###

namenlijst = ["AnnaZoet"]#, "ChrisKrommendijk"]#, "LiekeZwier", "MartZoet", "ThijsBink", "BasvanderKaaden", "CorentinMonat", "TomMeulenkamp", "DylanBruggeman", "WouterVisser"]
algorithms = ["Alg1", "Alg2"]
markers_used = ["Ster_cen", "Ster_R", "Ster_L"]
N_interp = 100

import json
from collections import defaultdict
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import pandas as pd


def r_svd(base1, base2):
    base1_mean = np.mean(base1, axis=1, keepdims=True)
    base2_mean = np.mean(base2, axis=1, keepdims=True)
    base1_zero = base1 - base1_mean
    base2_zero = base2 - base2_mean
    u, _, vh = np.linalg.svd(base2_zero @ base1_zero.T)
    r = (u * np.array([1, 1, np.linalg.det(u @ vh)])) @ vh
    t = base2_mean - r @ base1_mean
    return r, t, ((base2 - (r @ base1 + t)) ** 2).mean() ** 0.5

def interpolate_trials(trials, N):
    output = []
    for trial in trials:
        if len(trial) < 2:
            continue
        f = interp1d(np.linspace(0, 1, num=len(trial)), trial, kind='linear')
        output.append(f(np.linspace(0, 1, num=N)))
    return np.array(output)

def extract_trials(name, algorithm):
    # padnamen
    json_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\qualisys_data\\{name}\\{name}_{algorithm}.json"
    csv_path = f"D:\\BMT\\Master\\Thesis\\Arduino\\prothese\\trial_data\\{name}\\{name}_trials_qualisys_{algorithm}.csv"
    
    # json
    with open(json_path, 'r') as f:
        json_data = json.load(f)

    marker_data = {}
    for marker in json_data["Markers"]:
        if marker["Name"] in markers_used:
            marker_data[marker["Name"]] = np.array(marker["Parts"][0]["Values"])[:, :3]

    n_frames = min(marker_data[m].shape[0] for m in markers_used)
    frames = [np.stack([marker_data[m][i] for m in markers_used], axis=1).T for i in range(n_frames)]

    trial_df = pd.read_csv(csv_path)
    trial_df.columns = trial_df.columns.str.strip()

    angles = defaultdict(list)
    trans = defaultdict(list)

    for _, row in trial_df.iterrows():
        start = int(float(row["Startframe"]))
        end = int(float(row["Eindframe"]))
        target = row["Target"]

        if start >= n_frames or end >= n_frames:
            continue

        base = frames[start].T
        angle_seq = []
        trans_seq = []

        for i in range(start + 1, end + 1):
            current = frames[i].T
            r, t, _ = r_svd(base, current)
            angle = np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))
            angle_seq.append(angle)
            trans_seq.append(np.linalg.norm(t))

        angles[target].append(angle_seq)
        trans[target].append(trans_seq)

    return angles, trans

pooled_angles = defaultdict(list)
pooled_trans = defaultdict(list)

for name in namenlijst:
    # baseline ophalen
    angles_null, trans_null = extract_trials(name, "Null")
    mean_angle_null = {k: np.mean(interpolate_trials(v, N_interp), axis=0) for k, v in angles_null.items()}
    mean_trans_null = {k: np.mean(interpolate_trials(v, N_interp), axis=0) for k, v in trans_null.items()}

    for algorithm in algorithms:
        angles_alg, trans_alg = extract_trials(name, algorithm)
        for target in angles_alg:
            if target not in mean_angle_null or target not in mean_trans_null:
                continue
            angles_interp = interpolate_trials(angles_alg[target], N_interp)
            trans_interp = interpolate_trials(trans_alg[target], N_interp)
            # rel_angle = angles_interp / mean_angle_null[target]
            rel_angle = angles_interp / np.interp(
            np.linspace(0, 1, N_interp),
            np.linspace(0, 1, len(mean_angle_null[target])),
            mean_angle_null[target]
)

            rel_trans = trans_interp / mean_trans_null[target]
            pooled_angles[(algorithm, target)].extend(rel_angle)
            pooled_trans[(algorithm, target)].extend(rel_trans)

    print(f"alle null data verwerkt voor {name}")

# --- Plot absolute waarden per trial voor NULL meting ---
angles_null, trans_null = extract_trials(name, "Null")

for target in sorted(angles_null.keys()):
    angles_interp = interpolate_trials(angles_null[target], N_interp)
    trans_interp = interpolate_trials(trans_null[target], N_interp)

    x = np.arange(N_interp)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(f"Null – {target} – Absolute waarden ({name})")

    # Rotatie
    for i, trial in enumerate(angles_interp):
        ax1.plot(x, trial, label=f"Trial {i+1}")
    ax1.set_ylabel("Rotatie (graden)")
    ax1.legend(loc='upper right', fontsize='small')
    ax1.set_ylim(-10,50)

    # Translatie
    for i, trial in enumerate(trans_interp):
        ax2.plot(x, trial, label=f"Trial {i+1}")
    ax2.set_ylabel("Translatie (mm)")
    ax2.set_xlabel("Genormaliseerde tijd")
    ax2.set_ylim(-20,200)

    plt.tight_layout()
    plt.show()


# --- Plot absolute waarden per trial vóór normalisatie ---
for algorithm in algorithms:
    for name in namenlijst:
        angles_alg, trans_alg = extract_trials(name, algorithm)
        for target in sorted(angles_alg.keys()):
            angles_interp = interpolate_trials(angles_alg[target], N_interp)
            trans_interp = interpolate_trials(trans_alg[target], N_interp)

            x = np.arange(N_interp)
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            fig.suptitle(f"{algorithm} – {target} – Absolute waarden") #  ({name})

            # Plot rotaties
            for i, trial in enumerate(angles_interp):
                ax1.plot(x, trial, label=f"Trial {i+1}")
            ax1.set_ylabel("Rotatie (graden)")
            ax1.legend(loc='upper right', fontsize='small')
            ax1.set_ylim(-10,50)

            # Plot translaties
            for i, trial in enumerate(trans_interp):
                ax2.plot(x, trial, label=f"Trial {i+1}")
            ax2.set_ylabel("Translatie (mm)")
            ax2.set_xlabel("Genormaliseerde tijd")
            ax2.set_ylim(-20,200)

            plt.tight_layout()
            plt.show()

##### genormaliseerde data
for algorithm in algorithms:
    for target in sorted(set(k[1] for k in pooled_angles.keys())):
        angle_mat = np.vstack(pooled_angles[(algorithm, target)])
        trans_mat = np.vstack(pooled_trans[(algorithm, target)])
        mean_angle = np.mean(angle_mat, axis=0)
        std_angle = np.std(angle_mat, axis=0)
        mean_trans = np.mean(trans_mat, axis=0)
        std_trans = np.std(trans_mat, axis=0)

        print("tijd om te plotten")

        x = np.arange(N_interp)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax1.set_title(f"{algorithm} – Target {target} – Gepoolde gemiddelde ± std")
        ax1.plot(x, mean_angle, label="mean")
        ax1.fill_between(x, mean_angle - std_angle, mean_angle + std_angle, alpha=0.3)
        ax1.set_ylabel("Rel. rotatie")

        ax2.plot(x, mean_trans, label="mean")
        ax2.fill_between(x, mean_trans - std_trans, mean_trans + std_trans, alpha=0.3)
        ax2.set_ylabel("Rel. translatie")
        ax2.set_xlabel("Genormaliseerde tijd")

        plt.tight_layout()
        plt.show()

print("klaar met het script")
