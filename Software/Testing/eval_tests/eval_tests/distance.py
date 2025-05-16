import numpy as np


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
tests = [test_1, test_2, test_3, test_4, test_5, test_6]

# --- Actual physical distance (ground truth) ---
actual_distance = np.array([6.0, 2.5, 6.0, 2.5])  # meters

# --- Compute distances between consecutive points ---
def segment_distances(positions):
    return np.linalg.norm(np.diff(positions, axis=0), axis=1)

# --- Store summary stats ---
summary = []

print("=======================================")
for i, test in enumerate(tests, 1):
    gv_mm = test[:, :2]   # first 2 columns
    slam_m = test[:, 2:]  # last 2 columns
    gv_m = gv_mm / 1000.0

    dist_gv = segment_distances(gv_m)
    dist_slam = segment_distances(slam_m)

    total_actual = np.sum(actual_distance)
    total_gv = np.sum(dist_gv)
    total_slam = np.sum(dist_slam)
    total_gv_error = abs(total_gv - total_actual)
    total_slam_error = abs(total_slam - total_actual)
    avg_gv_segment_error = np.mean(abs(dist_gv - actual_distance))
    avg_slam_segment_error = np.mean(abs(dist_slam - actual_distance))

    summary.append([
        i, total_gv, total_slam, total_gv_error, total_slam_error,
        avg_gv_segment_error, avg_slam_segment_error
    ])

    print(f"\n=== Test {i} ===")
    print("Segment\tActual(m)\tGV (m)\t\tSLAM (m)\tGV Error\tSLAM Error")
    for j in range(len(actual_distance)):
        err_gv = abs(dist_gv[j] - actual_distance[j])
        err_slam = abs(dist_slam[j] - actual_distance[j])
        print(f"{j+1}\t{actual_distance[j]:.2f}\t\t{dist_gv[j]:.3f}\t\t{dist_slam[j]:.3f}\t\t{err_gv:.3f}\t\t{err_slam:.3f}")

    print(f"\nTotal:  {total_actual:.2f} m\tGV: {total_gv:.3f} m\tSLAM: {total_slam:.3f} m")
    print(f"Total GV error: {total_gv_error:.3f} m")
    print(f"Total SLAM error: {total_slam_error:.3f} m")
print("=======================================\n")

# --- Final Summary Table ---
print("========== SUMMARY ==========")
print("Test\tGV Total\tSLAM Total\tGV Err\tSLAM Err\tGV AvgSegErr\tSLAM AvgSegErr")
for row in summary:
    test_id, gv_total, slam_total, gv_err, slam_err, avg_gv_seg, avg_slam_seg = row
    print(f"{test_id}\t{gv_total:.3f}\t\t{slam_total:.3f}\t\t{gv_err:.3f}\t{slam_err:.3f}\t\t{avg_gv_seg:.3f}\t\t{avg_slam_seg:.3f}")

# --- Optional: compute averages over all tests ---
summary_np = np.array(summary)
avg_row = np.mean(summary_np[:, 1:], axis=0)
print("\nAvg\t{:.3f}\t\t{:.3f}\t\t{:.3f}\t{:.3f}\t\t{:.3f}\t\t{:.3f}".format(*avg_row))
