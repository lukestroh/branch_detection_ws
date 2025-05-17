#!/usr/bin/env python3
import numpy as np
import plotly.graph_objects as go


def plot_ransac_quadratic_fit(
    zeroed_ts=None,
    fp_filter_data=None,
    raw_ts=None,
    raw_data=None,
    mav_filter_ts=None,
    mav_filter_data=None,
    t_fit=None,
    y_fit=None,
    inlier_mask=None,
    # start_window_time = None,
    # window_size = None,
    fig=None,
    show: bool = False,
    save_fig: bool = False,
    save_path: str = "",
) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    fig.update_xaxes(title="Time (s)")
    fig.update_yaxes(title="Distance (m)")

    if np.any(fp_filter_data):
        fig.add_trace(go.Scatter(x=zeroed_ts, y=fp_filter_data, name="maf-fpf", mode="lines"))

    if np.any(y_fit):
        fig.add_trace(go.Scatter(x=t_fit, y=y_fit, name="RANSAC fit", mode="lines"))

    if np.any(raw_data):
        fig.add_trace(go.Scatter(x=np.asarray(raw_ts) - raw_ts[0], y=raw_data, name="raw_sensor_data", mode="lines"))

    if np.any(mav_filter_data):
        fig.add_trace(
            go.Scatter(x=np.asarray(mav_filter_ts) - mav_filter_ts[0], y=mav_filter_data, name="MAF_data", mode="lines")
        )

    if np.any(inlier_mask):
        # Plot ransac masked data
        outlier_mask = np.logical_not(inlier_mask)
        fig.add_trace(go.Scatter(x=zeroed_ts[inlier_mask], y=fp_filter_data[inlier_mask], name="inliers", mode="lines"))
        fig.add_trace(
            go.Scatter(x=zeroed_ts[outlier_mask], y=fp_filter_data[outlier_mask], name="outliers", mode="lines")
        )

    # fig.add_vline(x=start_window_time, line_width=2, line_dash="dash", line_color='blue')
    # fig.add_vline(x=start_window_time+window_size, line_width=2, line_dash="dash", line_color='blue')
    if show:
        fig.show()
    # if save_fig:
    #     fig.write_image(save_path)
    return fig
