#!/usr/bin/env python3
import branch_detection_system_analysis.plot.debug_plots as dplot
import branch_detection_system_analysis.plot.plotting_backend as pb
import numpy as np
from numpy.typing import ArrayLike
import os
from pathlib import Path
import plotly.graph_objects as go
import plotly.io as pio
from rclpy.time import Time
import scipy.optimize as so
import sklearn.linear_model as sklm
import sklearn.preprocessing as skpp
import sklearn.metrics as skm
import traceback

import pprint as pp

from rclpy.node import Node

import py_trees


def far_plane_filter(
    filter_far_plane: float,
    ts: list,
    data: list,
) -> dict[str, np.ndarray] | None:
    """
    A low-pass filter, filters data points out beyond the `filter_far_plane` value. Resulting arrays are truncated.

    :param tof_far_plane: A user set parameter that can alter what is the maximum reading.
    :type tof_far_plane: float
    :param ts: Timestamp data
    :type maf_ts: list
    :param data: Distance data
    :type maf_data: list

    :return: A dictionary containing the filtered data
    :rtype: dict
    """
    # filter readings beyond a distance
    fp_filter_data = np.where(np.asarray(data) < filter_far_plane, data, np.nan)
    # Put all of those values beyond as np.nan
    fp_filter_ts = np.where(np.isnan(fp_filter_data), np.nan, np.asarray(ts))
    # Filter out the np.nan values
    fp_filter_data = fp_filter_data[~np.isnan(fp_filter_data)]
    fp_filter_ts = fp_filter_ts[~np.isnan(fp_filter_ts)]

    if fp_filter_ts.size == 0:
        # print(f"Could not find any data points less than the filter's far plane of {filter_far_plane}")
        return None

    fp_filter_zeroed_ts = fp_filter_ts - fp_filter_ts[0]

    return {"ts": fp_filter_ts, "ts_zeroed": fp_filter_zeroed_ts, "data": fp_filter_data, "tof_arm_radius": 0.4891}


def fit_ransac(
    window_data: dict,
    max_trials: int,
    min_samples: int,
    residual_threshold: float,
    node: Node = None,
):
    quadratic = skpp.PolynomialFeatures(degree=2)
    x = window_data["wrist_state"]
    # print(x   )
    try:
        x_quad = quadratic.fit_transform(X=x[:, np.newaxis])

        # Define RANSAC regressor
        ransac = sklm.RANSACRegressor(
            estimator=sklm.LinearRegression(),
            max_trials=max_trials,
            min_samples=min_samples,
            residual_threshold=residual_threshold,
        )
        ransac = ransac.fit(X=x_quad, y=window_data["data"])
    except ValueError as e:
        if node is not None:
            node.error(traceback.format_exc())
            # node.warn(min_samples)
        else:
            # print(traceback.format_exc())
            ...
        return None
    return ransac, quadratic, x_quad


def process_window(
    window_data: dict,
    ransac: sklm.RANSACRegressor,
    quadratic: skpp.PolynomialFeatures,
    xquad: np.ndarray,
    node: Node = None,
) -> dict:

    x = window_data["wrist_state"]
    # ts_zeroed = window_data['ts_zeroed']
    data = window_data["data"]

    x_fit = np.linspace(
        x[0],
        x[-1],
        len(x),
    )

    y_fit = ransac.predict(quadratic.fit_transform(x_fit[:, np.newaxis]))

    coefficients = ransac.estimator_.coef_
    fit_r2 = skm.r2_score(y_true=data, y_pred=ransac.predict(xquad))

    idx_min = np.argmin(y_fit)
    timestamp_min = window_data["ts"][idx_min]
    # node.warn(f"TIMESTAMP MIN: {timestamp_min}")
    fit_min = float(y_fit[idx_min])

    residuals = abs(data - y_fit)
    avg_residual = np.mean(residuals)

    window_data["coefficients"] = coefficients
    window_data["r2"] = fit_r2
    window_data["theta_min"] = window_data["wrist_state"][idx_min]
    window_data["ts_min"] = timestamp_min
    window_data["fit_min"] = fit_min
    window_data["avg_residual"] = avg_residual
    window_data["x_fit"] = x_fit
    window_data["y_fit"] = y_fit
    window_data["inlier_mask"] = ransac.inlier_mask_

    return window_data


def fit_and_process_ransac(
    window_data: dict,
    max_trials: int,
    min_samples: int,
    residual_threshold: float,
    node: Node = None,
):
    fit_res = fit_ransac(
        window_data=window_data,
        max_trials=max_trials,
        min_samples=min_samples,
        residual_threshold=residual_threshold,
        node=node,
    )
    if fit_res is not None:
        ransac, quadratic, xquad = fit_res
        result = process_window(
            window_data=window_data,
            ransac=ransac,
            quadratic=quadratic,
            xquad=xquad,
            node=node,
        )
    else:
        result = _error_result(
            window_id=window_data["window_id"],
            start_time=window_data["start_ts"],
            window_size=window_data["window_size"],
        )

    return result


def window_ransac(
    data: dict,
    window_size: float,
    # window_step: float,
    window_overlap_ratio: float,
    max_trials: int,
    min_samples: int,
    residual_threshold: float,
    debug_plot: bool = False,
    node: Node = None,
) -> dict:
    """
    :param data: Dictionary of all trial data
    :type dict:
    :param window_size: Time interval of the moving window
    :type window_size: float
    :param window_overlap_ratio: Ratio of the current window with which to overlap the next window.
    :type window_overlap_ratio: float
    :param max_trials: Maximum RANSAC trials allowed for each window.
    :type max_trials: int
    :param min_samples: Minimum number of samples required for each window.
    :type min_samples: int
    :param residual_threshold: Residual threshold for the RANSAC window.
    :type residual_threshold: float
    :param debug_plot: If true, plots the windowed data.
    :type debug_plot: bool
    :param node: ros2 Node object. Used for logging, default None.
    :type node: rclpy.node.Node

    :returns: A dictionary containing all of the windowed data. If errors are thrown due to the set RANSAC parameters, dictionary values will be None.
    :rtype: dict
    """
    ts_window_start = 0.0
    ts_window_end = ts_window_start + window_size

    ts_trial_end = data["ts_zeroed"][-1]

    window_idx = 0
    last_window_reached = False
    window_results = {}

    while not last_window_reached:
        # Prune window size at end
        if ts_window_start + window_size <= ts_trial_end:
            ts_window_end = ts_window_start + window_size
        else:
            ts_window_end = ts_trial_end
            last_window_reached = True

        window_indices = np.where((data["ts_zeroed"] >= ts_window_start) & (data["ts_zeroed"] < ts_window_end))
        # t_window = data["ts"][window_indices]
        # zt_window = data["ts_zeroed"][window_indices]
        # dist_window = data["data"][window_indices]

        # data['start_ts']
        # data['window_size'] = window_size

        window = {}
        window["window_id"] = window_idx
        window["start_ts"] = ts_window_start
        window["window_size"] = window_size
        window["ts"] = data["ts"][window_indices]
        window["ts_zeroed"] = data["ts_zeroed"][window_indices]
        window["data"] = data["data"][window_indices]
        window["wrist_state"] = data["wrist_state"][window_indices]
        window["rotation_speed"] = np.pi / 16  # rad / s TODO: get from topic, publish from controller
        window["tof_arm_radius"] = 0.4891

        window = fit_and_process_ransac(
            window_data=window,
            max_trials=max_trials,
            min_samples=min_samples,
            residual_threshold=residual_threshold,
            node=node,
        )

        window_results[window_idx] = window
        if window_overlap_ratio > 0:
            ts_window_start += window_size * (1 - window_overlap_ratio)
        else:
            ts_window_start += window_size
        window_idx += 1

    return window_results


def parabola(x, a, b, c):
    return a * x**2 + b * x + c


def find_parabola_height_and_width(window: dict, verbose: bool = False, node: Node = None) -> dict:
    """
    Given a parabola y = ax^2 + bx + c, find x values where y(x_min + w/2) = y(x_min) + h, where w is the width of the parabola and h is the height from y_min. We can solve directly for h given the desired point of h = w/2

    :param window: A dictionary containing metadata about the parabolic fit.
    :type window: dict
    :returns: An updated dictionary with parabolic fit width and height
    :rtype: dict
    """
    a = window["coefficients"][2]
    b = window["coefficients"][1]
    c = window["coefficients"][0]
    r = window["tof_arm_radius"]
    x_min = window["ts_min"] - window["ts"][0]
    y_min = window["fit_min"]

    rotation_speed = window["rotation_speed"]

    # One root is always zero, the other is:
    h_root = (r**2 - r * (2 * a * x_min + b)) / a
    hy_at_root = parabola(x=h_root, a=a, b=b, c=c)
    height = abs(hy_at_root - y_min) * r
    width = height * 2
    d_theta = width / r

    if verbose:
        if node is not None:
            node.info(f"fit COEFS: {a}, {b}, {c}")
            node.info(f"fit x_min: {x_min}")
            node.info(f"fit y_min: {y_min}")
            # node.info(f"h_func COEFS: {h_coefs}")
            node.info(f"h_func ROOTS: {h_root}")
            node.info(f"HEIGHTS: {height}")
            node.info(f"WIDTHS: {width}")
        else:
            print(f"fit COEFS: {a}, {b}, {c}")
            print(f"fit x_min: {x_min}")
            print(f"fit y_min: {y_min}")
            # print(f"h_func COEFS: {h_coefs}")
            print(f"h_func ROOTS: {h_root}")
            print(f"HEIGHTS: {height}")
            print(f"WIDTHS: {width}")

    window["height"] = height
    window["width"] = width
    window['d_theta'] = d_theta
    return window


def filter_parabola_by_max_diameter(window: dict, max_diameter: float = 0.0508):
    if not np.isclose(window["height"], window["width"] / 2, atol=0.001):
        window["branch_candidate"] = False
        # print(1)
    elif window["height"] < 0 or window["height"] > max_diameter / 2:
        window["branch_candidate"] = False
        # print(2)
        # print(window['height'])
    elif window["width"] < 0 or window["width"] > max_diameter:
        window["branch_candidate"] = False
        # print(3)
    else:
        window["branch_candidate"] = True
        # print(4)

    return window


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

        # derivative = [2 * coefs[2], coefs[1]]
        # roots = np.roots(p=derivative)
        # root = roots[0]  # quadratic derivative should just have 1 root
        # if the root falls outside of the window, remove it
        # if (root < window["start_ts"]) or (root > window["start_ts"] + window["window_size"]):
        #     # print("REJECTED!")
        #     continue

        window = find_parabola_height_and_width(window=window, verbose=False)
        window = filter_parabola_by_max_diameter(window=window, max_diameter=0.0508)
        if not window["branch_candidate"]:
            continue

        filtered_window_data.update({window_idx: window})

    # pp.pprint(filtered_window_data)

    return filtered_window_data


def _error_result(window_id, start_time, window_size):
    return {
        "window_id": window_id,
        "start_ts": start_time,
        "window_size": window_size,
        "ts": None,
        "ts_zeroed": None,
        "data": None,
        "coefficients": None,
        "r2": None,
        "ts_min": None,
        "fit_min": None,
        "t_fit": None,
        "y_fit": None,
        'height': None,
        'width': None,
        'd_theta': None,
        "inlier_mask": None,
        "avg_residual": None,
        "tof_arm_radius": None,
        "rotation_speed": None,
        "wrist_state": None,
    }


def group_overlapping_parabolas(window_data: dict) -> dict:
    window_ids = list(window_data.keys())
    split_idxs = np.where(np.diff(window_ids) > 10)[0] + 1
    grouped_ids = np.split(window_ids, split_idxs)

    grouped_windows = {}

    for i, group in enumerate(grouped_ids):
        grouped_windows[i] = []
        for widx in group:
            grouped_windows[i].append({widx: window_data[widx]})
    return grouped_windows


def refit_by_group(data: dict, group: list[dict], node: Node = None):
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
        node=node,
        t_window=t_window,
        zt_window=zt_window,
        dist_window=dist_window,
        start_ts=start_ts,
        window_size=cropped_window_size,
        max_trials=20,
        min_samples=10,
        residual_threshold=0.004,
    )

    return result


def get_branch_center_time_and_distance(
    df_dict: dict,
    filter_far_plane: float,
    sensor_name: str,
    debug_plot: bool = False,
    save_plot: bool = False,
    save_plot_path: str = None,
    window_size: float = 0.75,
    window_step_size: float = 0.25,
    window_overlap_ratio: float = 1 / 3,
    min_samples: int = 5,
    max_trials: int = 20,
    residual_threshold: float = 0.004,
    node: Node = None,
) -> dict[int, dict]:
    """
    1. Filter the data, timestamps.
    2. Get the joint states at those timesteps
    3. Fit ransac to the distance vs. joint state
    """
    best_window = None
    try:
        raw_ts = df_dict[f"{sensor_name}_raw"][f"{sensor_name}_raw_ts"]
        raw_data = df_dict[f"{sensor_name}_raw"][f"{sensor_name}_raw_data"]
        maf_ts = df_dict[f"{sensor_name}_filtered"][f"{sensor_name}_filtered_ts"]
        maf_data = df_dict[f"{sensor_name}_filtered"][f"{sensor_name}_filtered_data"]

        ####################################################################################
        # # TODO: DELETE FOR LIVE RUN
        # sliced_ts = maf_ts[:len(maf_ts)//3]
        # sliced_ts = sliced_ts[len(sliced_ts)//3:]
        # sliced_data = maf_data[:len(maf_data)//3]
        # sliced_data = sliced_data[len(sliced_data)//3:]

        fpf_data = far_plane_filter(filter_far_plane=filter_far_plane, ts=maf_ts, data=maf_data)
        # fpf_data = far_plane_filter(filter_far_plane=filter_far_plane, ts=sliced_ts, data=sliced_data)

        ###################################################################################
        if fpf_data is None:
            if node is not None:
                node.error(f"{sensor_name}: Failed at fpf_filter()")
            else:
                print(f"{sensor_name}: Failed at fpf_filter()")
            return None

        joint_states_df = pb.get_df_rows_at_closest_timestamp(
            df_dict=df_dict, topic_name="joint_states", timestamps=fpf_data["ts"]
        )
        joint_states_numpy = joint_states_df.to_numpy()
        joint_states_ts = joint_states_numpy[:, 0]
        fpf_data["wrist_state"] = joint_states_numpy[:, 1]

        # print(fpf_data)

        windowed_data = window_ransac(
            data=fpf_data,
            window_size=window_size,
            window_overlap_ratio=window_overlap_ratio,
            max_trials=max_trials,
            min_samples=min_samples,
            residual_threshold=residual_threshold,
            node=node,
        )

        # node.debug(f"{windowed_data}")

        filtered_window_data = filter_parabolas(windowed_data=windowed_data)
        if not filtered_window_data:
            if node is not None:
                node.error(f"{sensor_name}: Failed at filter_parabolas()")
            else:
                print(f"{sensor_name}: Failed at filter_parabolas()")
            return None

        avg_residual_start = 0.0
        for window in filtered_window_data.values():
            if window["avg_residual"] > avg_residual_start:
                best_window = window
                avg_residual_start = window["avg_residual"]
            # if node is not N
            # print(best_window)

        # grouped_window_data = group_overlapping_parabolas(window_data=filtered_window_data)
        # if not grouped_window_data:
        #     if node is not None:
        #         node.warn(f"{sensor_name}: Failed at group_overlapping_parabolas()")
        #     else:
        #         print(f"{sensor_name}: Failed at group_overlapping_parabolas()")
        #     return None

        # refit_data = {}
        # for group_id, group in grouped_window_data.items():
        #     refit_data.update({group_id: refit_by_group(data=fpf_data, group=group, node=node)})

        # pp.pprint(filtered_window_data)
    finally:
        if debug_plot:
            fig = dplot.plot_maf_vs_joint_state(
                data=fpf_data,
                sensor_name=sensor_name,
            )
            for window in windowed_data.values():
                fig = dplot.plot_ransac_tof_vs_joint_state(
                    data=window, sensor_name=sensor_name, description="all_windows", fig=fig
                )
            fig.show()

            fig = dplot.plot_maf_vs_joint_state(
                data=fpf_data,
                sensor_name=sensor_name,
            )
            for window_id, window in filtered_window_data.items():
                fig = dplot.plot_ransac_tof_vs_joint_state(
                    data=window, sensor_name=sensor_name, description="filtered_windows", fig=fig
                )
            fig.show()

            if best_window is not None:
                fig = dplot.plot_maf_vs_joint_state(
                    data=fpf_data,
                    sensor_name=sensor_name,
                )
                fig = dplot.plot_ransac_tof_vs_joint_state(
                    data=best_window, sensor_name=sensor_name, description="best_window", fig=fig
                )
                fig.show()

        # # final fit parabola
        # fig = dplot.plot_ransac_quadratic_fit(
        #     zeroed_ts=fpf_data["ts_zeroed"],
        #     fp_filter_data=fpf_data["data"],
        #     raw_ts=raw_ts,
        #     raw_data=raw_data,
        #     # mav_filter_ts=mav_filter_ts,
        #     # mav_filter_data=mav_filter_data,
        # )
        # # for window_id, window in refit_data.items():
        # #     fig = dplot.plot_ransac_quadratic_fit(t_fit=window["t_fit"], y_fit=window["y_fit"], fig=fig)
        # if save_plot:
        #     pio.write_image(fig=fig, file=os.path.join(save_plot_path, f"{sensor_name}_refit.svg"), format="svg")
        #     pio.write_html(fig=fig, file=os.path.join(save_plot_path, f"{sensor_name}_refit.html"), auto_open=True)

    # # Get the lowest residual fit
    # lowest_residual = np.inf
    # lowest_residual_idx = None
    # for refit_idx, refit in refit_data.items():
    #     if refit["avg_residual"] is not None:
    #         if refit["avg_residual"] < lowest_residual:
    #             lowest_residual = refit["avg_residual"]
    #             lowest_residual_idx = refit_idx

    # if lowest_residual == np.inf or lowest_residual_idx is None:
    #     return None
    # else:
    #     ts_min = refit_data[lowest_residual_idx]["ts_min"]
    #     fit_min = refit_data[lowest_residual_idx]["fit_min"]

    ts_min = best_window["ts_min"]
    fit_min = best_window["fit_min"]

    return ts_min, fit_min
