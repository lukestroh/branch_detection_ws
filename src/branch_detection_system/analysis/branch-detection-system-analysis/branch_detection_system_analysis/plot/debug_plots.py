#!/usr/bin/env python3
from branch_detection_system_analysis.plot import plotly_helpers as ph

import numpy as np
from numpy.typing import ArrayLike
import os
import plotly.graph_objects as go
import plotly.io as pio

import pprint as pp


def plot_branch_projection(
    tof0: ArrayLike,
    tof1: ArrayLike,
    branch_center_pos: ArrayLike,
    branch_vec_ori: ArrayLike,
    desired_eef_pos: ArrayLike,
    desired_eef_ori: ArrayLike,
    plot_base_origin: bool = True,
    base_origin_name: str = None,
    save_fig: bool = False,
    save_fig_dir: str = None,
    fig: go.Figure = None,
) -> go.Figure:
    if fig is None:
        fig = go.Figure()

        if plot_base_origin:
            fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], name=f"{base_origin_name}__base"))

        fig.add_trace(
            go.Scatter3d(
                x=[tof0[0]],
                y=[tof0[1]],
                z=[tof0[2]],
                name="tof0_proj",
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=[tof1[0]],
                y=[tof1[1]],
                z=[tof1[2]],
                name="tof1_proj",
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=[branch_center_pos[0]],
                y=[branch_center_pos[1]],
                z=[branch_center_pos[2]],
                name="branch_center",
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=[desired_eef_pos[0]],
                y=[desired_eef_pos[1]],
                z=[desired_eef_pos[2]],
                name="desired_eef_xyz",
            )
        )
        ph.plot_vector(
            fig=fig,
            position=desired_eef_pos,
            orientation=desired_eef_ori,
            scale=0.5,
            color="blue",
            anchor="tail",
        )
        ph.plot_vector(
            fig=fig,
            position=desired_eef_pos,
            orientation=branch_vec_ori,
            scale=0.5,
            color="red",
            anchor="tail",
        )
        fig.update_layout(title=dict(text="Projected branch detection points"), scene=dict(aspectmode="data"))
        if save_fig:
            pio.write_image(fig=fig, file=os.path.join(save_fig_dir, "projections.svg"), format="svg")
            pio.write_html(fig=fig, file=os.path.join(save_fig_dir, "projections.html"), auto_open=True)

    return fig


def plot_maf_vs_joint_state(
    data: dict, sensor_name: str, fig: go.Figure = None, save_fig: bool = False, save_path: str = ""
):
    if fig is None:
        fig = go.Figure()


    fig.add_trace(
        go.Scatter(
            x=data["wrist_state"],
            y=data["data"],
            mode="lines",
            text=data["ts"],
            hovertemplate="theta: %{x}<br>d: %{y}<br>time: %{text}<br>r2: %{r2}<extra></extra>",
        )
    )

    return fig


def plot_ransac_tof_vs_joint_state(
    data: dict, sensor_name: str, fig: go.Figure = None, save_fig: bool = False, save_path: str = ""
):
    # if data['']
    if fig is None:
        fig = go.Figure()

    # print(data)
    

    fig.add_trace(
        go.Scatter(
            x=data["wrist_state"],
            y=data["y_fit"],
            name=sensor_name,
            text=data["ts"],
            hovertemplate="theta: %{x}<br>d: %{y}<br>time: %{text}<extra></extra>",
        )
    )

    fig.update_xaxes(title="wrist3 position (rad)")
    fig.update_yaxes(title="Distance (m)")
    fig.update_layout(title=dict(text="ToF raw, filtered, and windowed fits vs. Wrist 3 joint state"))

    if save_fig:
        pio.write_html(fig=fig, file=save_path)

    return fig


def plot_ransac_tof_vs_timestamp():

    return


def plot_ransac_quadratic_fit(
    sensor_name: str = None,
    joint_states_data=None,
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
    fig.update_xaxes(title="wrist3_pos (rad)")
    fig.update_yaxes(title="Distance (m)")

    if sensor_name is not None:
        fig.update_layout(
            title=dict(text=f"{sensor_name} RANSAC fits"),
            scene=dict(camera=dict(center=dict(x=1, y=-1, z=2), eye=dict(x=-0.5, y=1, z=-0.5)), aspectmode="data"),
        )

    if np.any(joint_states_data) and np.any(fp_filter_data):
        fig.add_trace(go.Scatter(x=joint_states_data[2], y=fp_filter_data, name="tof vs. wrist3", mode="lines"))

    # if np.any(fp_filter_data):
    #     fig.add_trace(go.Scatter(x=zeroed_ts, y=fp_filter_data, name="maf-fpf", mode="lines"))

    if np.any(y_fit):
        fig.add_trace(go.Scatter(x=t_fit, y=y_fit, name="RANSAC fit", mode="lines"))

    if np.any(raw_data):
        fig.add_trace(go.Scatter(x=np.asarray(raw_ts) - raw_ts[0], y=raw_data, name="raw_sensor_data", mode="lines"))

    if np.any(mav_filter_data):
        fig.add_trace(go.Scatter(x=np.asarray(mav_filter_ts), y=mav_filter_data, name="MAF_data", mode="lines"))

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
