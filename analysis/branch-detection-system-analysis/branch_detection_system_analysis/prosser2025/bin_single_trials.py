#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates
from branch_detection_system_analysis.prosser2025 import curve_fitting as cf
from branch_detection_system_analysis.prosser2025 import plotly_helpers as ph
import glob
import numpy as np
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
import plotly.subplots
import os
from scipy.spatial.transform import Rotation
import scipy.optimize as so
import traceback

import pprint as pp
import sys


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))


# with pd.HDFStore("tf2_transforms.h5", "r") as h5f:
#     df_timestamp = h5f["transforms"].query("timestamp == 1700000000.123456")
# print(df_timestamp)


def get_files_by_iteration(trial_num: int) -> list[str]:
    trial_num_str = str(trial_num).zfill(3)
    return glob.glob(warehouse_path + f"/**/*{trial_num_str}.h5")


def get_files_by_topic(topic: str):
    return glob.glob(warehouse_path + f"/**/*{topic}*.h5")


def get_files_by_topics(topics: list[str]) -> list[str]:
    files = []
    for topic in topics:
        files += get_files_by_topic(topic=topic)
    return files


def get_files_by_topics_by_trial_name(topics: list[str], trial_name: str) -> list[str]:
    files = []
    for topic in topics:
        files += glob.glob(
            warehouse_path + f"/**/*{trial_name}*{topic}*.h5"
        )  # NOTE: This includes tf_static both times
    return files


def filter_files_by_trial_number(files: list[str], trial_number: int) -> list[str]:
    return [file for file in files if file.endswith(f"{str(trial_number).zfill(3)}.h5")]


#####################################################################
# def get_df_from_h5(h5_path: str) -> pd.DataFrame:
#     df = pd.read_hdf(path_or_buf=h5_path)
#     return df


def get_topic_name_from_filename(filename: str):
    return filename.split("__")[-2]


def build_df_dict_from_files(data_dict: dict, files: list[str]) -> None:
    for file in files:
        topic_name = get_topic_name_from_filename(filename=file)
        df = pd.read_hdf(path_or_buf=file)
        data_dict.update({topic_name: df})
    return


def get_tf_df_at_closest_timestamp(tf_df: pd.DataFrame, tf_static_df: pd.DataFrame, timestamp: float):
    """Gets the closest set of TF frames at a given timestamp"""
    ts_closest = tf_df.iloc[(tf_df["tf_ts"] - timestamp).abs().argsort()[:1]]
    tf_dynamic_df = tf_df.loc[tf_df["tf_ts"] == ts_closest["tf_ts"].item()]
    tf_all_links_df = pd.DataFrame(
        data=np.vstack([tf_static_df.values, tf_dynamic_df.values]), columns=tf_dynamic_df.columns
    )
    return tf_all_links_df


def get_tf_matrix_from_df(target_frame: str, source_frame: str, tf_df: pd.DataFrame) -> np.ndarray:
    # NOTE: This only goes forwards right now.
    # ur5e__base_link_inertia
    # ur5e__ft_frame
    transformation_mat = np.identity(4)
    frame_to_frame_mat = np.identity(4)

    source_frame_parent = tf_df.loc[tf_df["tf_child_frame_id"] == source_frame, ["tf_frame_id"]]["tf_frame_id"].iloc[0]
    # print(source_frame_parent)

    # print(tf_df)

    while True:
        try:
            if target_frame == source_frame_parent:
                tf_target_to_child_df = tf_df.loc[
                    (tf_df["tf_frame_id"] == target_frame) & (tf_df["tf_child_frame_id"] == source_frame)
                ]
            else:
                tf_target_to_child_df = tf_df.loc[
                    (tf_df["tf_frame_id"] == target_frame)
                    & (~tf_df["tf_child_frame_id"].isin(["ur5e__base", "ur5e__ft_frame"]))
                ]

            frame_to_frame_mat[:3, 3] = [
                tf_target_to_child_df["tf_t_x"].iloc[0],
                tf_target_to_child_df["tf_t_y"].iloc[0],
                tf_target_to_child_df["tf_t_z"].iloc[0],
            ]
            frame_to_frame_mat[:3, :3] = Rotation.from_quat(
                [
                    tf_target_to_child_df["tf_r_x"].iloc[0],
                    tf_target_to_child_df["tf_r_y"].iloc[0],
                    tf_target_to_child_df["tf_r_z"].iloc[0],
                    tf_target_to_child_df["tf_r_w"].iloc[0],
                ]
            ).as_matrix()

            transformation_mat = transformation_mat @ frame_to_frame_mat

            target_frame = tf_target_to_child_df["tf_child_frame_id"].iloc[0]
            if target_frame == source_frame:
                break

        except Exception as e:
            print(traceback.format_exc())
            break

    return transformation_mat


def split_trial_by_fpc_deactivate(
    df: pd.DataFrame, df_topic_name: str, transition_event_df: pd.DataFrame, trial_num: int
) -> tuple[pd.DataFrame]:
    split_time = transition_event_df.at[trial_num * 2 + 1, "controller_transition_events_ts"]

    df_search_for_branch = df.loc[df[f"{df_topic_name}_ts"] <= split_time]
    df_align_and_approach_branch = df.loc[df[f"{df_topic_name}_ts"] > split_time]

    return df_search_for_branch, df_align_and_approach_branch


def group_df_by_parabola(df_search_action: pd.DataFrame, topic_name: str):
    ts_gap_threshold = 0.5
    df_search_action[f"{topic_name}_ts_diff"] = df_search_action[f"{topic_name}_ts"].diff()
    df_search_action["group"] = (df_search_action[f"{topic_name}_ts_diff"] > ts_gap_threshold).cumsum()

    return df_search_action


def plot_tof_trial(
    data: pd.DataFrame, topic_name: str, trial_num: int, start_time: float = 0.0, fig: go.Figure = None
) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=data[f"{topic_name}_ts"] - start_time,
            y=data[f"{topic_name}_data"],
            mode="markers",
            name=f"{topic_name}__{trial_num}",
        )
    )
    fig.update_layout(title=dict(text=f"{topic_name}__{trial_num}"))
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Distance (m)")
    return fig


def plot_transition_event(
    transition_events_df: pd.DataFrame,
    start_time: float,
    start_index: int,
    mid_trial_index: int,
    end_index: int,
    color: str,
    fig: go.Figure = None,
) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    timestamp = transition_events_df.at[start_index, "controller_transition_events_ts"] - start_time
    fig.add_vline(x=timestamp, line_width=2, line_dash="dash", line_color=color)
    fig.add_annotation(
        x=timestamp,
        y=0,
        text=f"transition_event_{start_index}",
        showarrow=True,
        xanchor="left",
        yanchor="middle",
        textangle=-90,
    )

    if mid_trial_index is not None:
        timestamp = transition_events_df.at[mid_trial_index, "controller_transition_events_ts"] - start_time
        fig.add_vline(x=timestamp, line_width=2, line_dash="dash", line_color=color)
        fig.add_annotation(
            x=timestamp,
            y=0,
            text=f"transition_event_{mid_trial_index}",
            showarrow=True,
            xanchor="left",
            yanchor="middle",
            textangle=-90,
        )

    timestamp = transition_events_df.at[end_index, "controller_transition_events_ts"] - start_time
    fig.add_vline(x=timestamp, line_width=2, line_dash="dash", line_color=color)
    fig.add_annotation(
        x=timestamp,
        y=0,
        text=f"transition_event_{end_index}",
        showarrow=True,
        xanchor="left",
        yanchor="middle",
        textangle=-90,
    )

    return fig


def plot_all_individual_trials():
    #########################################################
    # Plot all individual trials
    if data_dict["tof0_filtered"]["tof0_filtered_ts"].iloc[0] <= data_dict["tof1_filtered"]["tof1_filtered_ts"].iloc[0]:
        time_begin = data_dict["tof0_filtered"]["tof0_filtered_ts"].iloc[0]
    else:
        time_begin = data_dict["tof1_filtered"]["tof1_filtered_ts"].iloc[0]

    fig = plot_tof_trial(
        data=data_dict["tof0_filtered"], start_time=time_begin, topic_name="tof0_filtered", trial_num=i
    )
    fig = plot_tof_trial(
        data=data_dict["tof1_filtered"], start_time=time_begin, topic_name="tof1_filtered", trial_num=i, fig=fig
    )
    fig = plot_transition_event(
        transition_events_df=df_fpc_transition_events,
        start_time=time_begin,
        start_index=start_index,
        mid_trial_index=mid_trial_index,
        end_index=end_index,
        color="green",
        fig=fig,
    )
    fig = plot_transition_event(
        transition_events_df=df_sjtc_transition_events,
        start_time=time_begin,
        start_index=start_index,
        mid_trial_index=mid_trial_index,
        end_index=end_index,
        color="red",
        fig=fig,
    )
    fig.show()

    i += 1
    if i == 13:
        start_index = end_index
        mid_trial_index = None
        end_index = start_index + 1
    else:
        start_index = end_index
        mid_trial_index = start_index + 1
        end_index = start_index + 2
    ################################################################
    return


def fit_3d_linear_pca(points):
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    # Use SVD to find line passing through "middle" of data
    # U can be used to reconstruct how the original points project onto the principal directions.
    # S contains the amount of variance along each direction.
    # Vt contains the principal directions of your data:
    #   Vt[0] is the direction of maximum variance — the dominant direction your data stretches in.
    U, S, Vt = np.linalg.svd(centered_points)
    direction = Vt[0]
    direction = direction / np.linalg.norm(direction)

    return centroid, direction


def compute_linear_residuals(points, centroid, direction):
    line_direction_norm = direction / np.linalg.norm(direction)
    centered_points = points - centroid
    projections = centered_points @ line_direction_norm
    closest_points = centroid + np.outer(projections, line_direction_norm)
    residuals = np.linalg.norm(points - closest_points, axis=1)
    # print(residuals)
    return


def fit_3d_quadratic(points, centroid, direction):
    """Project points onto a line to get parameter t-values and quadratic coefficients"""

    deltas = points - centroid
    t_vals = (
        deltas @ direction  # / np.linalg.norm(direction) # direction vec is normalized
    )  # Does the dot product -- projection of each delta onto the direction vector
    quadratic_design_mat = np.column_stack([t_vals**2, t_vals, np.ones_like(t_vals)])

    # Fit x(t), y(t), z(t)
    coefs, resids, rank, _ = np.linalg.lstsq(quadratic_design_mat, points[:, 0:3], rcond=None)
    coefs = coefs.T
    return t_vals, coefs


def compute_quadratic_residuals(points, t_vals, coefs):
    quadratic_design_mat = np.vstack([t_vals**2, t_vals, np.ones_like(t_vals)])
    fitted_points = coefs @ quadratic_design_mat
    deltas = points[:, 0:3].T - fitted_points
    residuals = np.linalg.norm(deltas, axis=0)
    return residuals


def evaluate_quadratic(t, coefs):
    return np.array(
        [
            coefs[0, 0] * t**2 + coefs[0, 1] * t + coefs[0, 2],
            coefs[1, 0] * t**2 + coefs[1, 1] * t + coefs[1, 2],
            coefs[2, 0] * t**2 + coefs[2, 1] * t + coefs[2, 2],
        ]
    )


def evaluate_quadratic_derivative(t, coefs):
    return np.array(
        [2 * coefs[0, 0] * t + coefs[0, 1], 2 * coefs[1, 0] * t + coefs[1, 1], 2 * coefs[2, 0] * t + coefs[2, 1]]
    )


def get_orthogonality(t, point, coefs):
    t = float(np.squeeze(t))
    curve_point = evaluate_quadratic(t=t, coefs=coefs)
    tangent = evaluate_quadratic_derivative(t=t, coefs=coefs)

    residual = point - curve_point
    print(np.linalg.norm(np.dot(residual, tangent)))
    return np.linalg.norm(np.dot(residual, tangent))


def get_dist_to_curve(t_val, point, coefs):
    t_val = float(np.squeeze(t_val))
    curve_point = evaluate_quadratic(t=t_val, coefs=coefs)
    # return np.linalg.norm(curve_point - point)
    return np.sum((curve_point - point) ** 2)


def project_points_onto_curve(points, t_vals, coefs):
    """Project the points onto the curve and return the t_value"""
    ortho_t_vals = []
    projected_points = []

    for i, point in enumerate(points[:, 0:3]):

        # res = so.minimize(fun=get_dist_to_curve, x0=[t_vals[i]], args=(point, coefs))
        # print(evaluate_quadratic(t=res.x[0], coefs=coefs))
        # ortho_t_vals.append(res.x)
        # projected_points.append(evaluate_quadratic(t=res.x[0], coefs=coefs))

        res = so.fmin(
            get_orthogonality,
            [t_vals[i]],
            args=(
                point,
                coefs,
            ),
            disp=True,
            full_output=True,
            xtol=1e-10,
            ftol=1e-12,
            maxfun=1000,
            maxiter=1000,
        )
        print(res)
        ortho_t_vals.append(res[0][0])
        projected_points.append(evaluate_quadratic(t=res[0][0], coefs=coefs))
        # import sys
        # sys.exit()

        # p_proj = evaluate_quadratic(res[0][0], coefs)
        # tangent = evaluate_quadratic_derivative(res[0][0], coefs)
        # residual = point - p_proj
        # dot = np.dot(residual, tangent)
        # print(dot)

        # for i, point in enumerate(points[:, 0:3]):

        #     def dist_to_curve(t):
        #         curve_point = evaluate_quadratic(t, coefs=coefs)
        #         return np.linalg.norm(curve_point - point)

        # res = so.minimize_scalar(
        #     get_dist_to_curve, args=(point, coefs), bounds=(t_vals[i] - 0.1, t_vals[i] + 0.1), method="bounded"
        # )
        # print(res.x)
        # ortho_t_vals.append(res.x)
        # projected_points.append(evaluate_quadratic(t=res.x, coefs=coefs))

    # break

    return np.array(ortho_t_vals), np.array(projected_points)


def plot_quadratic_fit(t_vals: np.ndarray, coefs: np.ndarray, fig: go.Figure = None):
    if fig is None:
        fig = go.Figure()

    t_vals_plot = np.linspace(min(t_vals), max(t_vals), 100)
    # t_vals_plot = np.linspace(-1, 1, 500)
    x = coefs[0, 0] * t_vals_plot**2 + coefs[0, 1] * t_vals_plot + coefs[0, 2]
    y = coefs[1, 0] * t_vals_plot**2 + coefs[1, 1] * t_vals_plot + coefs[1, 2]
    z = coefs[2, 0] * t_vals_plot**2 + coefs[2, 1] * t_vals_plot + coefs[2, 2]
    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode="lines", name="quadratic fit"))

    return fig


def plot_quadratic_residuals(points, projected_points, fig: go.Figure = None):
    for p, q in zip(points, projected_points):
        fig.add_trace(
            go.Scatter3d(
                x=(p[0], q[0]),
                y=(p[1], q[1]),
                z=(p[2], q[2]),
                showlegend=False,
                mode="lines",
                line=dict(color="red"),
                legendgroup=0,
                legendgrouptitle={"text": "residuals"},
            )
        )

    return fig


def curve_derivative(t, coefs):
    return np.array(
        [2 * coefs[0, 0] * t + coefs[0, 1], 2 * coefs[1, 0] * t + coefs[1, 1], 2 * coefs[2, 0] * t + coefs[2, 1]]
    )


def main():

    data_dict = {}
    files_by_topics_by_trial_name = get_files_by_topics_by_trial_name(
        topics=[
            "tf",
            "tof0_raw",
            "tof1_raw",
            "tof0_filtered",
            "tof1_filtered",
            "fpc_transition_events",
            "sjtc_transition_events",
        ],
        trial_name="t1.2.1",
    )

    # Load trial-constant files
    tf_static_file = [file for file in files_by_topics_by_trial_name if file.endswith(f"tf_static.h5")][0]
    fpc_transition_events_file = [
        file for file in files_by_topics_by_trial_name if file.endswith(f"fpc_transition_events.h5")
    ][0]
    sjtc_transition_events_file = [
        file for file in files_by_topics_by_trial_name if file.endswith(f"sjtc_transition_events.h5")
    ][0]
    # print(sjtc_transition_events_file)
    # sys.exit()

    # Load dfs
    df_tf_static = pd.read_hdf(path_or_buf=tf_static_file)
    df_fpc_transition_events = pd.read_hdf(path_or_buf=fpc_transition_events_file)
    df_sjtc_transition_events = pd.read_hdf(path_or_buf=sjtc_transition_events_file)
    df_fpc_transition_events = df_fpc_transition_events.loc[
        df_fpc_transition_events["controller_transition_start_state"]
        == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
    ].reset_index(drop=True)
    df_sjtc_transition_events = df_sjtc_transition_events.loc[
        df_sjtc_transition_events["controller_transition_start_state"]
        == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
    ].reset_index(drop=True)

    data_dict.update({"tf_static": df_tf_static})
    data_dict.update({"fpc_transition_events": df_fpc_transition_events})
    data_dict.update({"sjtc_transition_events": df_sjtc_transition_events})

    start_index = 0
    mid_trial_index = 1
    end_index = start_index + 2

    original_ransac_failure_count = 0
    original_ransac_success_count = 0
    tof0_world_points = []
    tof1_world_points = []

      
    return


if __name__ == "__main__":
    main()
