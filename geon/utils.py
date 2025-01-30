import logging
from pathlib import Path
import joblib.externals
import numpy as np
from scipy.stats import skew

import joblib
import os

from geon.model import GEON_MODEL
import torch
import scipy.stats as stats

import rospy

def create_log_dir(log_folder):
    if not log_folder.exists():
        log_folder.mkdir(parents=True, exist_ok=True)

    return log_folder


def create_logger(log_dir, log_file):

    log_dir = Path(log_dir)
    create_log_dir(log_dir)

    logger = logging.getLogger('geon_logger-v2')
    logger.setLevel(logging.INFO)

    log_path = log_dir / log_file
    handler = logging.FileHandler(log_path, mode='w')
    #formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    #formatter = logging.Formatter('%(levelname)s - %(message)s')
    formatter = logging.Formatter('%(message)s')
    handler.setFormatter(formatter)
    if logger.hasHandlers():
        logger.handlers.clear()
    logger.addHandler(handler)

    logger.info("getting started with logger")

    return logger


def create_logger_online(log_dir, log_file):

    log_dir = Path(log_dir)
    create_log_dir(log_dir)

    logger = logging.getLogger('simu_online')
    logger.setLevel(logging.INFO)

    log_path = log_dir / log_file
    handler = logging.FileHandler(log_path, mode='w')
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    if logger.hasHandlers():
        logger.handlers.clear()
    logger.addHandler(handler)

    logger.info("getting started with logger")

    return logger


def statistic(data):
    q1 = np.percentile(data, 25)
    q3 = np.percentile(data, 75)
    iqr_val = q3 + (q3 - q1) * 1.5
    med_val = np.median(data)

    mean_val = np.mean(data)
    std_val = np.std(data)
    z1_val = mean_val + std_val
    z2_val = mean_val + std_val * 2

    if std_val < 1e-8:
        skew_val = 0
    else:
        skew_val = stats.skew(data)

    hist, bin_edges = np.histogram(data, bins=256, range=(0, 256), density=True)
    hist = hist[hist > 0]
    entropy_val = -np.sum(hist * np.log2(hist))

    result = {
        "iqr" : iqr_val,
        "med" : med_val,
        "mean": mean_val,
        "std" : std_val,
        "skew": skew_val,
        "entropy": entropy_val,
        "z1": z1_val,
        "z2": z2_val
    }

    return result


def adjust_data_size(data, num_points=128):
    num_existing_points = data.shape[0]

    #np.random.seed(42)

    if num_existing_points > num_points:
        indices = np.random.choice(num_existing_points, num_points, replace=False)
        data = data[indices, :]
    else:
        indices = np.random.choice(num_existing_points, num_points - num_existing_points, replace=True)
        repeated_points = data[indices, :]
        data = np.vstack((data, repeated_points))

    return data


def load_model1(model1_file):
    try:
        if os.path.exists(model1_file):
            return joblib.load(model1_file)
        raise FileNotFoundError(f"File {model1_file} not found.")
    except (FileNotFoundError, joblib.externals.process_executor.TerminatedWorkerError) as e:
        rospy.logerr(f"Error loading model1: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error occurred with model1: {e}")
    return None


def load_model2(model2_file):
    try:
        model2 = GEON_MODEL(k=10, num_classes=8)
        #stat_dict = torch.load(model2_file, weights_only=True)

        stat_dict = torch.load(
            model2_file,
            weights_only=True,
            encoding='latin1',
            map_location=torch.device('cpu')
        )

        model2.load_state_dict(stat_dict)
        model2 = model2.to("cpu")
        model2.eval()
        return model2
    except (FileNotFoundError, torch.nn.modules.module.ModuleAttributeError, RuntimeError) as e:
        rospy.logerr(f"Error loading model2: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error occurred with model2: {e}")
    return None


def load_scaler1(scaler_file):
    try:
        if os.path.exists(scaler_file):
            return joblib.load(scaler_file)
        raise FileNotFoundError(f"Scaler file {scaler_file} not found.")
    except (FileNotFoundError, joblib.externals.process_executor.TerminatedWorkerError) as e:
        rospy.logerr(f"Error loading scaler: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error occurred with scaler: {e}")
    return None


def get_feature_factors(data):
    mean_val = np.mean(data)
    stddev_val = np.std(data)

    if stddev_val < 1e-8:
        skewness_val = kurtosis_val = 0
    else:
        skewness_val = stats.skew(data)
        kurtosis_val = stats.kurtosis(data)

    hist, bin_edges = np.histogram(data, bins=256, range=(0, 256), density=True)
    hist = hist[hist > 0]
    entropy_val = -np.sum(hist * np.log2(hist))

    contrast_val = np.max(data) - np.min(data)

    result = {
        "mean": mean_val,
        "std": stddev_val,
        "skewness": skewness_val,
        "kurtosis": kurtosis_val,
        "entropy": entropy_val,
        "contrast": contrast_val,
    }

    return result
