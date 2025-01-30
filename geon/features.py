import numpy as np
import geon.utils as gu

from sklearn.cluster import DBSCAN

import rospy

#np.set_printoptions(precision=2, suppress=True)

def trim_by_ranges(data, x_range, z_range):
    return (data[:, 0] >= x_range[0]) & (data[:, 0] <= x_range[1]) & (data[:, 2] >= z_range[0]) & (data[:, 2] <= z_range[1])


def trim_by_threshold(values):
    stat = gu.statistic(values)
    threshold_skew = 2.6
    threshold_entropy = 5.0

    threshold_value = stat.get('z1')
    if (stat.get('skew') > threshold_skew or stat.get('entropy') > threshold_entropy) and stat.get('z2') < 255:
        threshold_value = stat.get('z2')

    mask = values > threshold_value

    return mask


def refine_labels(trimmed_pcd, n_clusters, labels, stat):
    v_avg = stat.get("mean")
    v_std = stat.get("std")
    threshold_skew = -1
    threshold = v_avg - v_std if stat.get("skew") < threshold_skew else v_avg

    cndt_labels = []
    val_list = []
    for i in range(n_clusters):
        labled_pcd = trimmed_pcd[labels == i]
        means = np.mean(labled_pcd, axis=0)
        if means[-1] >= threshold:
            cndt_labels.append(i)
            val_list.append(means[-1])

    return cndt_labels, val_list


def clustering_DBSCAN(trimmed_pcd, eps, min_points):
    intensity = trimmed_pcd[:,-1]
    data = np.column_stack((trimmed_pcd[:, 0:2], intensity/255))
    model = DBSCAN(eps=eps, min_samples=min_points)
    model.fit(data)

    labels = model.labels_
    n_clusters = labels.max().item() + 1

    return n_clusters, labels


def choose_best_labels(labels, vals, threshold):
    best_labels = []
    cnt = len(labels)
    max_v = max(vals) - threshold

    for i in range(cnt):
        if vals[i] > max_v:
            best_labels.append(labels[i])

    return best_labels


def estimate_coord_hat(data):
    consts = np.array([0.7, 0.7, 1.2]) + 0.1

    meds = np.median(data[:,0:3], axis=0)
    rangesbw = np.array([[meds[0] - consts[0]/2, meds[0] + consts[0]/2],
                         [meds[1] - consts[1]/2, meds[1] + consts[1]/2],
                         [meds[2] - consts[2]/2, meds[2] + consts[2]/2]])

    mins = np.min(data[:,0:3], axis=0)
    maxs = np.max(data[:,0:3], axis=0)
    diffs = np.abs(maxs - mins) > consts

    true_indices = np.where(diffs == True)[0]

    if len(true_indices) > 0:
        mask = np.all((data[:, true_indices] > rangesbw[true_indices, 0]) &
              (data[:, true_indices] < rangesbw[true_indices, 1]), axis=1)

        return data[mask]
    else:
        return data


def extract_feature_points(data, consts, min_points = 5):
    # step 1. trimming with constraints of ranges: x and z axies
    mask = trim_by_ranges(data, x_range=consts[0], z_range=consts[1])
    ranged_pcd = data[mask]

    # step 2. trimming with a values threshold
    intensity = ranged_pcd[:,-1]
    mask = trim_by_threshold(intensity)
    trimmed_pcd = ranged_pcd[mask]

    # check point
    if trimmed_pcd.shape[0] < min_points:
        rospy.logwarn(f"-- Can not process Clustering. {trimmed_pcd.shape = }")
        return None

    # step 3. clustering
    eps = 0.5 # 50cm
    n_clusters, labels = clustering_DBSCAN(trimmed_pcd, eps, min_points)
    if n_clusters < 0:
        return None

    # step 4. refinements
    stat = gu.statistic(trimmed_pcd[:,-1])
    cndt_labels, val_list = refine_labels(trimmed_pcd, n_clusters, labels, stat)
    if len(cndt_labels) == 0:
        rospy.logwarn(f"-- the # of cndt_labels: {len(cndt_labels)}")
        return None

    if len(cndt_labels) > 1:
        v_std = stat.get("std")
        cndt_labels = choose_best_labels(cndt_labels, val_list, v_std)
        if len(cndt_labels) == 0:
            return None

    mask = np.isin(labels, cndt_labels)

    return estimate_coord_hat(trimmed_pcd[mask])
