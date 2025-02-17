#!/usr/bin/env python3

import numpy as np
import plotly.graph_objects as go
from rclpy.time import Time
import sklearn.linear_model as sklm
import sklearn.preprocessing as skpp
import sklearn.metrics as skm
import traceback

# from final_approach_controller.find_branch_roll_wrist_controller import FindBranchRollWristController


def parabola(x, a, b, c):
    return a * x**2 + b * x + c


def get_branch_center_time_and_distance(
    node,
    raw_timestamps: list,
    filtered_timestamps: list,
    raw_readings: list,
    filtered_readings: list,
    sensor_name: str,
    debug_plot: bool = False,
    recursion_depth: int = 0
):
    if recursion_depth >= 2:
        return None
    # Clean data
    try:
        readings_plane_filtered = np.where(
            np.asarray(filtered_readings) < node.tof_far_plane, filtered_readings, np.nan
        )
        timestamps_filtered = np.where(np.isnan(readings_plane_filtered), np.nan, np.asarray(filtered_timestamps))
        readings_plane_filtered = readings_plane_filtered[~np.isnan(readings_plane_filtered)]
        timestamps_filtered = timestamps_filtered[~np.isnan(timestamps_filtered)]
        normalized_timestamps_filtered = timestamps_filtered - timestamps_filtered[0]
    except IndexError:
        node.info(f"No branch found for {sensor_name}")
        return None

    try:
        # Define RANSAC regressor
        ransac = sklm.RANSACRegressor(
            estimator=sklm.LinearRegression(), max_trials=300, min_samples=20, residual_threshold=0.004
        )

        # Fit RANSAC model to data
        quadratic = skpp.PolynomialFeatures(degree=2)
        x_quad = quadratic.fit_transform(X=normalized_timestamps_filtered[:, np.newaxis])
        ransac = ransac.fit(X=x_quad, y=readings_plane_filtered)

        # Get fitted curve
        t_fit = np.linspace(
            min(normalized_timestamps_filtered),
            max(normalized_timestamps_filtered),
            len(normalized_timestamps_filtered),
        )
        y_fit = ransac.predict(quadratic.fit_transform(t_fit[:, np.newaxis]))
    except Exception as e:

        node.error(traceback.format_exc())
        return None

    # Get r**2 value
    fit_r2 = skm.r2_score(y_true=readings_plane_filtered, y_pred=ransac.predict(x_quad))

    node.warn(f"{sensor_name} r^2: {fit_r2}")
    if fit_r2 <= 0.0:
        node.error(f"{sensor_name} r^2 value indicates a bad fit: {fit_r2}. Trying with half data.")
        
        raw_timestamps = raw_timestamps[len(raw_timestamps)//2 :]
        raw_readings = raw_readings[len(raw_readings)//2 :]
        filtered_timestamps = filtered_timestamps[len(filtered_timestamps)//2 :]
        filtered_readings = filtered_readings[len(filtered_readings)//2 :]
        return get_branch_center_time_and_distance(
            node=node,
            raw_timestamps=raw_timestamps,
            filtered_timestamps=filtered_timestamps,
            raw_readings=raw_readings,
            filtered_readings=filtered_readings,
            sensor_name=sensor_name,
            recursion_depth=1
        )
        

    idx_min = np.argmin(y_fit)
    timestamp_min = timestamps_filtered[idx_min]
    fit_min = float(y_fit[idx_min])
    split_time = np.modf(timestamp_min)
    time_center = Time(seconds=int(split_time[1]), nanoseconds=split_time[0] * 1e9)

    if debug_plot:
        fig = go.Figure()
        fig.add_trace(
            go.Scatter(
                x=normalized_timestamps_filtered,
                y=readings_plane_filtered,
                name="filtered_data",
            )
        )
        fig.add_trace(go.Scatter(x=t_fit, y=y_fit, name="RANSAC fit"))
        fig.add_trace(
            go.Scatter(x=np.asarray(raw_timestamps) - raw_timestamps[0], y=raw_readings, name="raw_sensor_data")
        )
        fig.add_trace(
            go.Scatter(x=np.asarray(filtered_timestamps) - filtered_timestamps[0], y=filtered_readings, name="MAF_data")
        )

        # Plot ransac masked data
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        fig.add_trace(
            go.Scatter(
                x=normalized_timestamps_filtered[inlier_mask], y=readings_plane_filtered[inlier_mask], name="inliers"
            )
        )
        fig.add_trace(
            go.Scatter(
                x=normalized_timestamps_filtered[outlier_mask], y=readings_plane_filtered[outlier_mask], name="outliers"
            )
        )
        fig.update_layout(title=dict(text=sensor_name))
        fig.show()

    if time_center:
        node.info(f"Branch found at distance {fit_min} at time {timestamp_min} for {sensor_name}")

    return time_center, fit_min
