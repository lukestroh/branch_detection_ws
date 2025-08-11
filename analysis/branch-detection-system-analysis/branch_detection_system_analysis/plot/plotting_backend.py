#!/usr/bin/env python3
from collections import defaultdict
import datetime
import glob
import numpy as np
from numpy.typing import ArrayLike
import os
import pandas as pd
import plotly.graph_objects as go
import re
from typing import Callable
from scipy.spatial.transform import Rotation

from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates

import pprint as pp

import traceback


class FileManager:
    def __init__(self):
        return


# ================================
#    Data collection functions
# ================================
class PlotFig(go.Figure):
    def __init__(self):
        layout = dict(legend_title_font_size=20)
        super().__init__(layout=layout)
        return


def get_db_by_trial_name(storage_path: str, name: str) -> list[str]:
    files = glob.glob(os.path.join(storage_path, name) + f"/{name}_0.db3.zstd")
    return files


def get_files_by_trial_name(warehouse_path: str, name: str) -> list[str]:
    files = glob.glob(os.path.join(warehouse_path, name + "_0") + "/*.h5")
    return files


def get_files_by_topic(warehouse_path: str, topic: str):
    return glob.glob(warehouse_path + f"/**/*{topic}*.h5")


def get_files_by_topics(warehouse_path: str, topics: list[str]) -> list[str]:
    files = []
    for topic in topics:
        files += get_files_by_topic(warehouse_path=warehouse_path, topic=topic)
    return files


def get_files_by_datetime(warehouse_path: str, _datetime: datetime.datetime) -> list[str]:
    files = glob.glob(warehouse_path + f"/**/*{datetime.datetime.strftime(_datetime, format=r'%Y%m%d')}*.h5")
    return files


def get_files_by_date(warehouse_path: str, date: str):
    files = glob.glob(warehouse_path + f"/**/*{date}*.h5")
    return files


# =============================
#    File filtering functions
# =============================
def filter_files_by_trial_number(files: list[str], trial_number: int) -> list[str]:
    """WARNING: Only for multi-trial use"""
    return [file for file in files if file.endswith(f"{str(trial_number).zfill(3)}.h5")]


def filter_files_by_topic(files: list[str], topic: str) -> list[str]:
    """Filters a list of files by topic and sorts them. Returns an empty list if no filenames match the topic."""
    match = re.fullmatch(r"[A-Za-z0-9_.\-/]+", topic)
    if match is None:
        return []
    else:
        pattern = rf"__{re.escape(topic)}__"
        file_list = [f for f in files if re.search(pattern, f)]
        return sorted(file_list)


def filter_files_by_topics(files: list[str], topics: list[str]) -> list[str]:
    _files = []
    for topic in topics:
        _files.extend(filter_files_by_topic(files, topic))
    return _files


# ===================================
#    DataFrame filtering functions
# ===================================
def filter_transition_events_for_controller_deactivating(df: pd.DataFrame):
    df_transition_events = df.loc[
        (
            df["controller_transition_goal_state"].fillna(-1).astype(int)
            == TransitionStates.TRANSITION_STATE_DEACTIVATING.value
        )
    ].reset_index()

    return df_transition_events


def filter_transition_events_for_controller_inactive(df: pd.DataFrame):
    df_transition_events = df.loc[
        (df["controller_transition_goal_state"].fillna(-1).astype(int) == TransitionStates.PRIMARY_STATE_INACTIVE.value)
    ].reset_index()

    return df_transition_events


def filter_transition_events_for_controller_activating(df: pd.DataFrame):
    df_transition_events = df.loc[
        (
            df["controller_transition_goal_state"].fillna(-1).astype(int)
            == TransitionStates.TRANSITION_STATE_ACTIVATING.value
        )
    ].reset_index()

    return df_transition_events


def filter_transition_events_for_controller_active(df: pd.DataFrame):
    df_transition_events = df.loc[
        (df["controller_transition_goal_state"].fillna(-1).astype(int) == TransitionStates.PRIMARY_STATE_ACTIVE.value)
    ].reset_index()

    return df_transition_events


# ==========================
#    Metadata functions
# ==========================
def extract_file_metadata(filename: str) -> dict:
    location_datetime_match = re.search(r"bds__(?P<location>.+?)__(?P<datetime>\d{8}_\d{2}-\d{2}-\d{2})_", filename)
    topic_match = re.search(r"__([a-zA-Z0-9_]+)__\d+\.h5$", filename)

    return {
        "filename": filename,
        "location": location_datetime_match.group("location") if location_datetime_match else None,
        "datetime": location_datetime_match.group("datetime") if location_datetime_match else None,
        "topic": topic_match.group(1) if topic_match else None,
    }


# ==========================
#    Grouping functions
# ==========================
def group_metadata(metadata: list[dict], *grouping_keys: str | Callable[[dict], str]) -> dict:
    """
    Groups metadata by one or more keys. Keys can be strings or callables.
    """

    def _get_group_value(entry, key):
        return key(entry) if callable(key) else entry.get(key)

    def _recursive_group(entries, keys):
        if not keys:
            if len(entries) == 1:
                return entries[0]
        grouped = defaultdict(list)
        key_fn = keys[0]
        for entry in entries:
            val = _get_group_value(entry=entry, key=key_fn)
            grouped[val].append(entry)
        return {k: _recursive_group(v, keys[1:]) for k, v in grouped.items()}

    return _recursive_group(entries=metadata, keys=grouping_keys)


def group_files_by_datetime(files: list[str]) -> defaultdict[str, str]:
    grouped_by_datetime = defaultdict(list)
    datetime_pattern = re.compile(r"__(\d{8})_(\d{2})-(\d{2})-(\d{2})_")
    for file in files:
        match = datetime_pattern.search(file)
        if match:
            datetime_str = match.group(0).strip("_")
            grouped_by_datetime[datetime_str].append(file)
    return grouped_by_datetime


def group_files_by_datetime_by_topic(files: list[str]) -> defaultdict[str, defaultdict[str, str]]:
    grouped_by_datetime_by_topic = defaultdict(lambda: defaultdict(list))

    datetime_pattern = re.compile(r"__(\d{8})_(\d{2})-(\d{2})-(\d{2})_")
    topic_pattern = re.compile(r"__([a-zA-Z0-9_]+)__\d+\.h5$")

    for file in files:
        datetime_match = datetime_pattern.search(file)
        topic_match = topic_pattern.search(file)

        if datetime_match and topic_match:
            datetime_str = datetime_match.group(0).strip("_")
            topic_str = topic_match.group(1)
            grouped_by_datetime_by_topic[datetime_str][topic_str].append(file)
    return grouped_by_datetime_by_topic


# =================================
#    Data organization functions
# =================================
def get_topic_name_from_filename(filename: str):
    return filename.split("__")[-2]


def build_df_dict_from_files(data_dict: dict | None, files: list[str]) -> dict:
    if data_dict is None:
        data_dict = {}
    for file in files:
        topic_name = get_topic_name_from_filename(filename=file)
        df = pd.read_hdf(path_or_buf=file)
        data_dict.update({topic_name: df})
    return data_dict


def get_df_rows_at_closest_timestamp_from_df_dict(
    df_dict: dict, topic_name: str, timestamps: ArrayLike
) -> pd.DataFrame:
    # """Gets the closest set of TF frames at a given timestamp"""
    df = df_dict[topic_name]
    return get_df_rows_at_closest_timestamp(df=df, topic_name=topic_name, timestamps=timestamps)


def get_df_rows_at_closest_timestamp(df: pd.DataFrame, topic_name: str, timestamps: ArrayLike):
    ts_col = df[f"{topic_name}_ts"].to_numpy()
    idxs = np.searchsorted(ts_col, timestamps)

    # Clip to avoid index errors
    idxs = np.clip(idxs, 1, len(ts_col) - 1)

    # Compare to previous timestamp for closeness
    prev = ts_col[idxs - 1]
    next_ = ts_col[idxs]
    prev_diff = np.abs(prev - timestamps)
    next_diff = np.abs(next_ - timestamps)

    # Use prev if it's closer
    closest_idxs = np.where(prev_diff < next_diff, idxs - 1, idxs)

    return df.iloc[closest_idxs].reset_index(drop=True)


def get_list_rows_at_closest_timestamps(data_dict: dict, topic_name: str, timestamps: ArrayLike) -> np.ndarray:
    topic_dict = data_dict[topic_name]
    ts = np.asarray(topic_dict[f"{topic_name}_ts"])
    data = np.asarray(topic_dict[f"{topic_name}_pos"])
    idxs = np.searchsorted(ts, timestamps)

    # Clip to avoid index errors)
    idxs = np.clip(idxs, 1, len(ts) - 1)

    # Compare to previous timestamp for closeness
    _prev = ts[idxs - 1]
    _next = ts[idxs]
    prev_diff = np.abs(_prev - timestamps)
    next_diff = np.abs(_next - timestamps)

    closest_idxs = np.where(prev_diff < next_diff, idxs - 1, idxs)

    return (ts[closest_idxs], data[closest_idxs])


def get_joint_angles_at_closest_timestamps(joint_angle_dict: dict, timestamps: ArrayLike) -> np.ndarray:
    ts = np.asarray(joint_angle_dict["ts"])
    data = np.asarray(joint_angle_dict["data"])
    idxs = np.searchsorted(ts, timestamps)

    # print(timestamps)

    # Clip to avoid index errors)
    idxs = np.clip(idxs, 1, len(ts) - 1)

    # Compare to previous timestamp for closeness
    _prev = ts[idxs - 1]
    _next = ts[idxs]
    prev_diff = np.abs(_prev - timestamps)
    next_diff = np.abs(_next - timestamps)

    closest_idxs = np.where(prev_diff < next_diff, idxs - 1, idxs)

    return (ts[closest_idxs], data[closest_idxs])


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

    i = 0
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

            # print(tf_target_to_child_df)

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

            # print(frame_to_frame_mat)
            # print(target_frame)
            # print(source_frame)
            # print(source_frame_parent)
            # if i == 1:
            #     import sys
            #     sys.exit()

            transformation_mat = transformation_mat @ frame_to_frame_mat

            target_frame = tf_target_to_child_df["tf_child_frame_id"].iloc[0]
            if target_frame == source_frame:
                break

            i += 1

        except Exception as e:
            print(traceback.format_exc())
            break

    # print(transformation_mat)
    # import sys
    # sys.exit()
    return transformation_mat


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Invert a 4x4 homogeneous transform matrix."""
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    T_inv = np.identity(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv


# ==========================
#    Plotting functions TODO: move to helpers
# ==========================
def plot_imu_data(imu_df: pd.DataFrame, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    fig.add_trace(go.Scatter(x=imu_df["imu_ts"], y=imu_df["imu_ax"], name="imu_ax"))
    fig.add_trace(go.Scatter(x=imu_df["imu_ts"], y=imu_df["imu_ay"], name="imu_ay"))
    fig.add_trace(go.Scatter(x=imu_df["imu_ts"], y=imu_df["imu_az"], name="imu_az"))
    return fig


def plot_linear_wrench_data(wrench_df: pd.DataFrame, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    fig.add_trace(go.Scatter(x=wrench_df["wrench_ts"], y=wrench_df["wrench_fx"], name="wrench_fx"))
    fig.add_trace(go.Scatter(x=wrench_df["wrench_ts"], y=wrench_df["wrench_fy"], name="wrench_fy"))
    fig.add_trace(go.Scatter(x=wrench_df["wrench_ts"], y=wrench_df["wrench_fz"], name="wrench_fz"))
    return fig


def plot_tof_vs_timestamp():
    return


def plot_quadratic_fit(t_vals: np.ndarray, coefs: np.ndarray, name: str = None, fig: go.Figure = None):
    if fig is None:
        fig = go.Figure()

    t_vals_plot = np.linspace(min(t_vals), max(t_vals), 100)

    # t_vals_plot = np.linspace(-1, 1, 500)
    x = coefs[0, 0] * t_vals_plot**2 + coefs[0, 1] * t_vals_plot + coefs[0, 2]
    y = coefs[1, 0] * t_vals_plot**2 + coefs[1, 1] * t_vals_plot + coefs[1, 2]
    z = coefs[2, 0] * t_vals_plot**2 + coefs[2, 1] * t_vals_plot + coefs[2, 2]
    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode="lines", line=dict(color="#5A4735", width=20), name=f"{name}"))

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
                line=dict(color="darkgoldenrod"),
                legendgroup=0,
                legendgrouptitle={"text": "residuals"},
            )
        )

    return fig
