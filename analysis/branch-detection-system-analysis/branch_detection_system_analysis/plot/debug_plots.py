#!/usr/bin/env python3
from branch_detection_system_analysis.plot import plotly_helpers as ph

import numpy as np
from numpy.typing import ArrayLike
import os
import plotly.graph_objects as go
import plotly.io as pio

import pprint as pp

_colors_dict = {"tof0": "#d8071f", "tof1": "#26a4d2", "all_data": "#85ce73", "s0": "#63b499", "s1": "#9c4ba7"}


def plot_tof_vs_joint_state(
    data: dict,
    name: str = "",
    save_fig: bool = False,
    save_path: str = None,
    fig: go.Figure = None,
):
    if fig is None:
        fig = go.Figure()
        fig.update_layout(title_text=f"{name}: tof vs. joint state")


    fig.add_trace(
        go.Scatter(
            x=data["joint_states_data"][:, 2],
            y=data["tof_data"],
            mode="markers",
            name=name,
            line=dict(color=_colors_dict[name]),
            customdata=np.column_stack(
                (
                    data["tof_ts"],
                    np.full(len(data["tof_ts"]), name),
                )
            ),
            hovertemplate="theta: %{x}<br>d: %{y}<br>time: %{customdata[0]}<br><extra>%{customdata[1]}</extra>",
        )
    )
    return fig


def plot_2d_tof_projection(
    data: dict, name: str = "", save_fig: bool = False, save_path: str = None, fig: go.Figure = None
):
    radius = 0.04891
    center = (0, 0)
    far_plane_filter = 0.20

    if fig is None:
        fig = go.Figure()
        fig = ph.plot_circle(center=center, radius=radius, name="tof_path", color="black", fig=fig)
        fig.update_layout(
            title_text=f"{name}: 2d projection onto ToF plane",
            xaxis=dict(title="x", scaleanchor="y", scaleratio=1),
            yaxis=dict(title="y", autorange="reversed"),
            scene=dict(aspectmode="data"),
            # plot_bgcolor="rgba(0,0,0,0)"
        )

    joint_states = np.where(np.asarray(data["tof_data"]) < far_plane_filter, data["joint_states_data"][:, 2], np.nan)
    joint_states = joint_states[~np.isnan(joint_states)]

    # Rotate the points to get them into eef view point
    x = radius * np.cos(joint_states)
    y = radius * np.sin(joint_states)
    xy = np.column_stack((x, y))
    rot_mat = np.array([[0, -1], [1, 0]])
    rotated_points = xy @ rot_mat
    fig.add_trace(
        go.Scatter(
            x=rotated_points[:, 0],
            y=rotated_points[:, 1],
            mode="markers",
            name=name,
            line=dict(color=_colors_dict[name]),
            customdata=np.column_stack(
                (
                    joint_states,
                    np.full(len(joint_states), name),
                )
            ),
            hovertemplate=("x: %{x}<br>" "y: %{y}<br>" "theta: %{customdata[0]}<br>" "<extra>%{customdata[1]}</extra>"),
        )
    )

    return fig


def plot_3d_tof_projection(
    data: dict, name: str = "", save_fig: bool = False, save_path: str = None, fig: go.Figure = None
):
    radius = 0.04891
    center = (0, 0)

    if fig is None:
        fig = go.Figure()
        fig = ph.plot_circle_3d(center=center, radius=radius, name="tof_path", color="black", fig=fig)
        fig.update_layout(
            title_text=f"{name}: 3d projection from ToF plane frame",
            xaxis=dict(title="x"),
            yaxis=dict(title="y"),
            # scene=dict(aspectmode="data"),
            # plot_bgcolor="rgba(0,0,0,0)"
        )

    far_plane_filter = 0.20

    joint_states = np.where(np.asarray(data["tof_data"]) < far_plane_filter, data["joint_states_data"][:, 2], np.nan)
    joint_states = joint_states[~np.isnan(joint_states)]

    distances = np.where(np.asarray(data["tof_data"]) < far_plane_filter, data["tof_data"], np.nan)
    distances = distances[~np.isnan(distances)]

    # Rotate the points to get them into eef view point
    x = radius * np.cos(joint_states)
    y = radius * np.sin(joint_states)
    xy = np.column_stack((x, y))
    rot_mat = np.array([[0, -1], [1, 0]])
    rotated_points = xy @ rot_mat
    fig.add_trace(
        go.Scatter3d(
            x=rotated_points[:, 0],
            y=rotated_points[:, 1],
            z=distances,
            mode="markers",
            marker=dict(size=4),
            name=name,
            line=dict(color=_colors_dict[name]),
            customdata=np.column_stack(
                (
                    joint_states,
                    np.full(len(joint_states), name),
                )
            ),
            hovertemplate=(
                "x: %{x}<br>"
                "y: %{y}<br>"
                "z: %{z}<br>"
                "theta: %{customdata[0]}<br>"
                "<extra>%{customdata[1]}</extra>"
            ),
        )
    )
    return fig


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
    """Plot both tof readings, a branch center point, and desired end-effetor pose. Helps understanding if alignment is true to reality.

    :param tof0: 3D projected point of tof0 reading.
    :type tof0: ArrayLike
    :param tof1: 3D projected point of tof1 reading.
    :type tof1: ArrayLike

    """
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

    eye_offset = np.array([-1, 2, 2])
    eye_pos = branch_center_pos - eye_offset
    fig.update_layout(
        title=dict(text="Projected branch detection points"),
        scene=dict(
            aspectmode="data",
            camera=dict(
                eye=dict(x=eye_pos[0], y=eye_pos[1], z=eye_pos[2]),
            ),
        ),
    )
    if save_fig:
        # pio.write_image(fig=fig, file=os.path.join(save_fig_dir, "projections.svg"), format="svg")
        pio.write_html(fig=fig, file=os.path.join(save_fig_dir, "projections.html"), auto_open=True)

    return fig


def plot_maf_vs_joint_state(
    data: dict, name: str, fig: go.Figure = None, save_fig: bool = False, save_path: str = ""
):
    if fig is None:
        fig = go.Figure()

    if data is None:
        return fig
    
    # pp.pprint(data)

    fig.add_trace(
        go.Scatter(
            x=data["wrist_state"],
            y=data["data"],
            mode="lines",
            # customdata=np.column_stack((data['ts'])),
            text=data["ts"],
            hovertemplate="theta: %{x}<br>d: %{y}<br>time: %{text}<br><extra></extra>",
        )
    )

    fig.update_xaxes(title="wrist3 position (rad)")
    fig.update_yaxes(title="Distance (m)")
    fig.update_layout(title=dict(text=f"{name} filtered and windowed fits vs. Wrist 3 joint state"))

    return fig


def plot_raw_vs_joint_state(data: dict, name: str, fig: go.Figure) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    if data is None:
        return fig
    return


def plot_ransac_tof_vs_joint_state(
    data: dict,
    name: str,
    fig: go.Figure = None,
    description: str = "",
    save_fig: bool = False,
    save_fig_path: str = "",
):
    if fig is None:
        fig = go.Figure()

    try:
        height = data["height"]
        width = data["width"]
    except KeyError:
        height = np.nan
        width = np.nan

    fig.add_trace(
        go.Scatter(
            x=data["wrist_state"],
            y=data["y_fit"],
            name=name,
            customdata=np.column_stack(
                (
                    np.full(len(data["ts"]), data["window_id"]),
                    data["ts"],
                    data["ts_zeroed"],
                    np.full(len(data["ts"]), data["coefficients"][2]),
                    np.full(len(data["ts"]), data["coefficients"][1]),
                    np.full(len(data["ts"]), data["coefficients"][0]),
                    np.full(len(data["ts"]), data["r2"]),
                    np.full(len(data["ts"]), data["avg_residual"]),
                    np.full(len(data["ts"]), height),
                    np.full(len(data["ts"]), width),
                )
            ),
            hovertemplate=(
                "id: %{customdata[0]}<br>"
                "theta: %{x}<br>"
                "d: %{y}<br>"
                "time: %{customdata[1]}<br>"
                "time_zeroed: %{customdata[2]}<br>"
                "y = %{customdata[3]}x<sup>2</sup> + %{customdata[4]}x + %{customdata[5]}<br>"
                "r<sup>2</sup>: %{customdata[6]}<br>"
                "avg_res: %{customdata[7]}<br>"
                "height: %{customdata[8]}<br>"
                "width: %{customdata[9]}<br>"
                "<extra></extra>"
            ),
        )
    )

    if description:
        fig.update_layout(title=dict(text=f"{name} {description} vs. Wrist 3 joint state"))

    if save_fig:
        pio.write_html(fig=fig, file=os.path.join(save_fig_path, f"{name}_{description}.html"), auto_open=True)

    return fig


def plot_ransac_tof_vs_timestamp():

    return


def plot_ransac_quadratic_fit(
    name: str = None,
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

    if name is not None:
        fig.update_layout(
            title=dict(text=f"{name} RANSAC fits"),
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
