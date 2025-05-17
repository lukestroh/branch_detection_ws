#!/usr/bin/env python3
import branch_detection_system_analysis.plot.debug_plots as dplot
import numpy as np
import os
import plotly.graph_objects as go
import plotly.io as pio
from rclpy.time import Time
import sklearn.linear_model as sklm
import sklearn.preprocessing as skpp
import sklearn.metrics as skm
import traceback

import pprint as pp

from rclpy.node import Node


def far_plane_filter(
    filter_far_plane: float,
    mav_filter_ts: list,
    mav_filter_data: list,
) -> dict[str, np.ndarray] | None:
    """
    :param tof_far_plane: A user set parameter that can alter what is the maximum reading.
    :type float:
    :param mav_filter_timestamps: Timestamp data from the moving average filter
    :type list:
    :param mav_filter_readings: Distance data from the moving average filter
    :type list:
    """
    # filter readings beyond a distance
    fp_filter_data = np.where(np.asarray(mav_filter_data) < filter_far_plane, mav_filter_data, np.nan)
    # Put all of those values beyond as np.nan
    fp_filter_ts = np.where(np.isnan(fp_filter_data), np.nan, np.asarray(mav_filter_ts))
    # Filter out the np.nan values
    fp_filter_data = fp_filter_data[~np.isnan(fp_filter_data)]
    fp_filter_ts = fp_filter_ts[~np.isnan(fp_filter_ts)]

    if fp_filter_ts.size == 0:
        # print(f"Could not find any data points less than the filter's far plane of {filter_far_plane}")
        return None

    fp_filter_zeroed_ts = fp_filter_ts - fp_filter_ts[0]

    return {
        "ts": fp_filter_ts,
        "zeroed_ts": fp_filter_zeroed_ts,
        "data": fp_filter_data,
    }


def fit_ransac(zt_window, dist_window, max_trials, min_samples, residual_threshold):
    quadratic = skpp.PolynomialFeatures(degree=2)
    x_quad = quadratic.fit_transform(X=zt_window[:, np.newaxis])

    # Define RANSAC regressor
    ransac = sklm.RANSACRegressor(
        estimator=sklm.LinearRegression(),
        max_trials=max_trials,
        min_samples=min_samples,
        residual_threshold=residual_threshold,
    )
    ransac = ransac.fit(X=x_quad, y=dist_window)
    return ransac, quadratic, x_quad


def process_window(start_time, window_size, t_window, zt_window, dist_window, ransac, quadratic, xquad) -> dict:

    t_fit = np.linspace(
        min(zt_window),
        max(zt_window),
        len(zt_window),
    )

    y_fit = ransac.predict(quadratic.fit_transform(t_fit[:, np.newaxis]))

    coefficients = ransac.estimator_.coef_
    fit_r2 = skm.r2_score(y_true=dist_window, y_pred=ransac.predict(xquad))

    idx_min = np.argmin(y_fit)
    timestamp_min = t_window[idx_min]
    fit_min = float(y_fit[idx_min])

    residuals = abs(dist_window - y_fit)
    avg_residual = np.mean(residuals)

    return {
        "start_ts": start_time,
        "window_size": window_size,
        "dist_window": dist_window,
        "coefficients": coefficients,
        "r2": fit_r2,
        "ts_min": timestamp_min,
        "fit_min": fit_min,
        "t_fit": t_fit,
        "y_fit": y_fit,
        "inlier_mask": ransac.inlier_mask_,
        "avg_residual": avg_residual,
    }


def _error_result(start_time, window_size):
    return {
        "start_ts": start_time,
        "window_size": window_size,
        "coefficients": None,
        "r2": None,
        "ts_min": None,
        "fit_min": None,
        "t_fit": None,
        "y_fit": None,
        "inlier_mask": None,
    }


def fit_and_process_ransac(
    zt_window, dist_window, start_ts, window_size, t_window, max_trials, min_samples, residual_threshold
):
    ransac, quadratic, xquad = fit_ransac(
        zt_window=zt_window,
        dist_window=dist_window,
        max_trials=max_trials,
        min_samples=min_samples,
        residual_threshold=residual_threshold,
    )
    result = process_window(
        start_time=start_ts,
        window_size=window_size,
        t_window=t_window,
        zt_window=zt_window,
        dist_window=dist_window,
        ransac=ransac,
        quadratic=quadratic,
        xquad=xquad,
    )
    return result


def window_ransac(
    data: dict,
    window_size: float,
    window_overlap_ratio: float,
    max_trials: int,
    min_samples: int,
    residual_threshold: float,
    debug_plot: bool = False,
) -> dict:
    """
    :param data: Dictionary of all trial data
    :type dict:

    """
    # start_window_time = timestamps_filtered[0]
    start_window_time = 0.0
    end_window_time = start_window_time + window_size
    # start_time = timestamps_filtered[0]
    # end_time = timestamps_filtered[-1] - start_time
    end_time = data["zeroed_ts"][-1]

    window_idx = 0
    last_window_reached = False
    window_results = {}

    while not last_window_reached:
        if start_window_time + window_size <= end_time:
            end_window_time = start_window_time + window_size
        else:
            end_window_time = end_time
            last_window_reached = True

        window_indices = np.where(
            (data["zeroed_ts"] >= start_window_time) & (data["zeroed_ts"] < start_window_time + window_size)
        )
        t_window = data["ts"][window_indices]
        zt_window = data["zeroed_ts"][window_indices]
        dist_window = data["data"][window_indices]

        # print(zt_window)

        try:
            ransac, quadratic, xquad = fit_ransac(
                zt_window=zt_window,
                dist_window=dist_window,
                max_trials=max_trials,
                min_samples=min_samples,
                residual_threshold=residual_threshold,
            )
            result = process_window(
                start_time=start_window_time,
                window_size=window_size,
                t_window=t_window,
                zt_window=zt_window,
                dist_window=dist_window,
                ransac=ransac,
                quadratic=quadratic,
                xquad=xquad,
            )
        except ValueError as e:
            # print(traceback.format_exc())
            result = _error_result(start_time=start_window_time, window_size=window_size)

        window_results[window_idx] = result
        if window_overlap_ratio > 0:
            start_window_time += window_size * (1 - window_overlap_ratio)
        else:
            start_window_time += window_size
        window_idx += 1

    return window_results


def filter_parabolas(windowed_data: dict) -> dict:
    filtered_window_data = {}
    for window_idx, window in windowed_data.items():
        coefs = window["coefficients"]
        r2 = window["r2"]
        if coefs is None:
            continue
        if coefs[2] < 0:
            continue
        if r2 is not None and r2 < 0:
            continue

        derivative = [2 * coefs[2], coefs[1]]
        roots = np.roots(p=derivative)
        root = roots[0]  # quadratic derivative should just have 1 root
        # if the root falls outside of the window, remove it
        if (root < window["start_ts"]) or (root > window["start_ts"] + window["window_size"]):
            # print("REJECTED!")
            continue

        filtered_window_data.update({window_idx: window})

    return filtered_window_data


def group_overlapping_parabolas(window_data: dict) -> dict:
    window_ids = list(window_data.keys())
    split_idxs = np.where(np.diff(window_ids) > 2)[0] + 1
    grouped_ids = np.split(window_ids, split_idxs)

    grouped_windows = {}

    for i, group in enumerate(grouped_ids):
        grouped_windows[i] = []
        for widx in group:
            grouped_windows[i].append({widx: window_data[widx]})
    return grouped_windows


def refit_by_group(data: dict, group: list[dict]):
    start_ts = np.inf
    end_ts = 0.0
    for window_data in group:
        for window in window_data.values():
            if window["start_ts"] < start_ts:
                start_ts = window["start_ts"]
            if window["start_ts"] + window["window_size"] > end_ts:
                end_ts = window["start_ts"] + window["window_size"]

    # Crop the window to 80%
    new_window_size = end_ts - start_ts
    cropped_window_size = (end_ts - start_ts) * 0.8
    start_ts = start_ts + (new_window_size - cropped_window_size) / 2
    end_ts = start_ts + cropped_window_size

    window_indices = np.where((data["zeroed_ts"] >= start_ts) & (data["zeroed_ts"] < end_ts))
    t_window = data["ts"][window_indices]
    zt_window = data["zeroed_ts"][window_indices]
    dist_window = data["data"][window_indices]

    result = fit_and_process_ransac(
        zt_window=zt_window,
        dist_window=dist_window,
        start_ts=start_ts,
        window_size=cropped_window_size,
        t_window=t_window,
        max_trials=20,
        min_samples=15,
        residual_threshold=0.004,
    )

    return result


def get_branch_center_time_and_distance(
    node: Node,
    filter_far_plane: float,
    raw_ts: list,
    raw_data: list,
    mav_filter_ts: list,
    mav_filter_data: list,
    sensor_name: str,
    debug_plot: bool = False,
    save_plot: bool = False,
    save_plot_path: str = None,
    window_size: float = 0.75,
    # window_step_size: float = 0.25,
    window_overlap_ratio: float = 1 / 3,
    min_samples: int = 5,
    max_trials: int = 20,
    residual_threshold: float = 0.004,
) -> dict[int, dict]:
    """
    TODO: Needs serious refactor, window size and overlap only relates to first bit, not the refitting params
    TODO: Alternatively, could record joint angles in their own array
    instead of tof vs time, match tof vs. wrist3

    """
    # The ransac regressor doesn't like the nans and the unix timestamps, so filter all data and normalize to first timestamp

    fpf_data = far_plane_filter(
        filter_far_plane=filter_far_plane, mav_filter_ts=mav_filter_ts, mav_filter_data=mav_filter_data
    )
    if fpf_data is None:
        node.warn("Failed at fpf_filter()")

        return None

    windowed_data = window_ransac(
        data=fpf_data,
        window_size=window_size,
        window_overlap_ratio=window_overlap_ratio,
        max_trials=max_trials,
        min_samples=min_samples,
        residual_threshold=residual_threshold,
    )
    # node.info(windowed_data)
    # print(pp.pformat(windowed_data))

    filtered_window_data = filter_parabolas(windowed_data=windowed_data)
    if not filtered_window_data:
        node.warn("Failed at filter_parabolas()")
        return None

    # def evaluate_parabola_shapes(windowed_data: dict):
    #     """Evaluate parabolic shapes for where the windows where the height = 1/2 the width. For good fits, this should be consistent

    #     :param windowed_data: Dictionary containing all of the windowed data, even for windows where no fit was found
    #     :type dict:
    #     """

    #     for window_idx, window in windowed_data.items():
    #         delta_t = 1 / window['coefficients'][2]
    #         print(window_idx, delta_t, window['t_fit'])
    #         # print(window['coefficients'][2])

    # evaluated_window_data = evaluate_parabola_shapes(windowed_data=filtered_window_data)

    # import sys
    # sys.exit()

    grouped_window_data = group_overlapping_parabolas(window_data=filtered_window_data)
    if not grouped_window_data:
        node.warn("Failed at group_overlapping_parabolas()")

        return None

    refit_data = {}
    for group_id, group in grouped_window_data.items():
        refit_data.update({group_id: refit_by_group(data=fpf_data, group=group)})

    if debug_plot:
        # if record_bag: TODO
        fig = dplot.plot_ransac_quadratic_fit(
            zeroed_ts=fpf_data["zeroed_ts"],
            fp_filter_data=fpf_data["data"],
            raw_ts=raw_ts,
            raw_data=raw_data,
            # mav_filter_ts=mav_filter_ts,
            # mav_filter_data=mav_filter_data,
        )

        for window_id, window in filtered_window_data.items():
            # pp.pprint(window)
            coefs = window["coefficients"]
            r2 = window["r2"]

            if coefs is None:
                continue
            if coefs[2] < 0:
                continue
            if r2 is not None and r2 < 0:
                continue
            derivative = [2 * coefs[2], coefs[1]]
            roots = np.roots(p=derivative)
            root = roots[0]  # quadratic derivative should just have 1 root
            # if the root falls outside of the window, remove it
            if (root < window["start_ts"]) or (root > window["start_ts"] + window["window_size"]):
                # print("REJECTED!")
                continue

            fig = dplot.plot_ransac_quadratic_fit(t_fit=window["t_fit"], y_fit=window["y_fit"], fig=fig)

        if save_plot:
            # file_loc = os.path.join(os.path.expanduser('~'), 'branch_detection_ws', 'analysis', 'branch-detection-system-analysis', 'branch_detection_system_analysis', 'figures', 'file.svg')
            pio.write_image(fig=fig, file=save_plot_path, format="svg")
        # fig.show()

    # Get the lowest residual fit
    lowest_residual = np.inf
    lowest_residual_idx = None
    for refit_idx, refit in refit_data.items():
        if refit["avg_residual"] < lowest_residual:
            lowest_residual = refit["avg_residual"]
            lowest_residual_idx = refit_idx
    ts_min = refit_data[lowest_residual_idx]["ts_min"]
    fit_min = refit_data[lowest_residual_idx]["fit_min"]

    return ts_min, fit_min
