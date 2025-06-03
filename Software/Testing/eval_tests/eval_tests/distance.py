import numpy as np
import matplotlib.pyplot as plt


# GulliView 2023
test_1 = np.array([
    [1570, 8141, -1.354812622070311, -0.7551897168159477],
    [1531, 1615, -7.055799491175673, -3.0033175030597734],
    [4464, 1723, -7.959382867979545, -0.5722727159330699],
    [4477, 8194, -2.1431332395152882, 1.743543573007466],
    [1572, 8147, -1.3298769313074303, -0.7372300811107917],
])  # in meters

test_2 = np.array([
    [1574, 8136, -1.370594382286059, -0.7194433808326651],
    [1538, 1616, -7.263774250934277, -2.90115503059126],
    [4473, 1713, -8.123766360057164, -0.46727575829801526],
    [4474, 8189, -2.0926024457375174, 1.8031757521375722],
    [1578, 8143, -1.3729100688840155, -0.7220918214251224],
])

test_3 = np.array([
    [1578, 8143, -1.3814903497695918, -0.6756795644760129],
    [1537, 1618, -7.21286708582916, -2.8483095462764347],
    [4478, 1720, -8.058489894496727, -0.39728198115744706],
    [4470, 8183, -2.1186259546974453, 1.816248342221789],
    [1577, 8137, -1.3861830474176515, -0.6794974678421891],
])

# GulliView 2025
test_4 = np.array([
    [6926, 1388, -1.4023399353027315, -0.5325517058372484],
    [943, 1351, -7.271248227501736, -2.69366819625922],
    [1027, 3786, -8.119908082741615, -0.24040039909506003],
    [7029, 3866, -2.14173373176497, 1.9704201259599623],
    [6926, 1388, -1.3923250384441803, -0.5285426773101118],
])

test_5 = np.array([
    [6926, 1387, -1.410170078277576, -0.420921564102159],
    [944, 1351, -7.283925394916454, -2.547976245065847],
    [1026, 3786, -8.165968992066778, -0.10731435190793435],
    [7032, 3868, -2.1230498307874064, 2.113226152654865],
    [6929, 1389, -1.409007782689549, -0.4170928884019434],
])
test_6 = np.array([
    [6929, 1390, -1.3859204053878753, -0.34409880638122503],
    [940, 1346, -7.252611999983386, -2.493438079882269],
    [1033, 3788, -8.085784092070217, -0.04551892212697319],
    [7041, 3862, -2.1031589811603175, 2.167551320101382],
    [6924, 1387, -1.3577524519649882, -0.33524470408288476],
])


# --- Test datasets (GV_mm_x, GV_mm_y, SLAM_x, SLAM_y) ---
tests23 = [test_1, test_2, test_3]
tests25 = [test_4, test_5, test_6]

# --- Actual physical distance (ground truth) ---
actual_distance = np.array([6.0, 2.5, 6.0, 2.5])  # meters

# --- Compute distances between consecutive points ---
def segment_distances(positions):
    return np.linalg.norm(np.diff(positions, axis=0), axis=1)

# --- Store summary stats ---
summary = []


def compute_errors(tests):
    gv_errors = []
    slam_errors = []
    for test in tests:
        slam = test[:, 2:]
        gv = test[:, :2] / 1000.0

        slam_dist = segment_distances(slam)
        gv_dist = segment_distances(gv)

        gv_pct_error = 100 * np.abs(gv_dist - actual_distance) / actual_distance
        slam_pct_error = 100 * np.abs(slam_dist - actual_distance) / actual_distance

        gv_errors.append(gv_pct_error)
        slam_errors.append(slam_pct_error)

    gv_errors = np.array(gv_errors)
    slam_errors = np.array(slam_errors)

    mean_gv_errors = np.mean(gv_errors, axis=0)
    mean_slam_errors = np.mean(slam_errors, axis=0)

    total_gv_error = np.mean(gv_errors)
    total_slam_error = np.mean(slam_errors)

    return mean_gv_errors, mean_slam_errors, total_gv_error, total_slam_error

# Compute
gv23_err, slam23_err, gv23_total, slam23_total = compute_errors(tests23)
gv25_err, slam25_err, gv25_total, slam25_total = compute_errors(tests25)

# Add "Total" to segment labels and error arrays
segments = [f"Segment {i+1}" for i in range(len(actual_distance))] + ["Totala sträckan"]
gv23_plot = np.append(gv23_err, gv23_total)
slam23_plot = np.append(slam23_err, slam23_total)
gv25_plot = np.append(gv25_err, gv25_total)
slam25_plot = np.append(slam25_err, slam25_total)

x = np.arange(len(segments))
width = 0.35

# --- Plot for GV 2023 ---
plt.figure(figsize=(10, 5))
plt.bar(x - width/2, gv23_plot, width, label='GulliView 2023')
plt.bar(x + width/2, slam23_plot, width, label='SLAM')
plt.xticks(x, segments)
plt.ylabel('Procentuell Avvikelse (%)')
plt.legend()
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.ylim(0, 18)
plt.tight_layout()
plt.show()

# --- Plot for GV 2025 ---
plt.figure(figsize=(10, 5))
plt.bar(x - width/2, gv25_plot, width, label='GulliView 2025')
plt.bar(x + width/2, slam25_plot, width, label='SLAM')
plt.xticks(x, segments)
plt.ylabel('Procentuell avvikelse (%)')
plt.legend()
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.ylim(0, 18)
plt.tight_layout()
plt.show()
