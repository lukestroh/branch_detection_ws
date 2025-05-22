#!/usr/bin/env python3
import datetime
import glob
import numpy as np
import os
import pandas as pd
import plotly.graph_objects as go
import re

"""
Data collection functions
"""


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
    files = glob.glob(warehouse_path + f"/**/*{datetime.datetime.strftime(_datetime, format=r'%Y%m%d_%H-%M-%S')}*.h5")
    return files


"""
Filtering functions
"""


def filter_files_by_trial_number(files: list[str], trial_number: int) -> list[str]:
    """WARNING: Only for multi-trial use"""
    return [file for file in files if file.endswith(f"{str(trial_number).zfill(3)}.h5")]


def filter_files_by_topic(files: list[str], topic: int) -> list[str] | None:
    match = re.fullmatch(r"[A-Za-z0-9_.-]+", topic)
    if match is None:
        return None
    else:
        pattern = rf"__{re.escape(topic)}__(?=)"
        return [f for f in files if re.search(pattern, f)]


def filter_files_by_topics(files: list[str], topics: list[str]) -> list[str] | None:
    _files = []
    for topic in topics:
        files_by_topic = filter_files_by_topic(files, topic)
        if files_by_topic:
            _files += files_by_topic
    return _files


"""
Data organization functions
"""


def get_topic_name_from_filename(filename: str):
    return filename.split("__")[-2]


def build_df_dict_from_files(data_dict: dict | None, files: list[str]) -> None:
    if data_dict is None:
        data_dict = {}

    for file in files:
        topic_name = get_topic_name_from_filename(filename=file)
        df = pd.read_hdf(path_or_buf=file)
        data_dict.update({topic_name: df})
    return data_dict


def get_df_rows_at_closest_timestamp(df_dict: dict, topic_name: str, timestamps: float) -> pd.DataFrame:
    # """Gets the closest set of TF frames at a given timestamp"""
    # ts_closest = df.iloc[(df[f"{topic_name}_ts"] - timestamp).abs().argsort()[:1]]
    # df_closest = df.loc[df[f"{topic_name}_ts"] == ts_closest[f"{topic_name}_ts"].item()]
    # return df_closest
    df = df_dict[topic_name]
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


"""
Plotting functions
"""


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


def plot_tof_vs_joint_state(df_dict: dict, tof_name: str, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    tof_df = df_dict[f"{tof_name}_filtered"]

    timestamps = tof_df[f"{tof_name}_filtered_ts"].to_numpy()

    joint_states_ts_filtered_df = get_df_rows_at_closest_timestamp(
        df_dict=df_dict, topic_name="joint_states", timestamps=timestamps
    )

    wrist_3_pos = np.vstack(joint_states_ts_filtered_df["joint_states_pos"])[:, 2]

    fig.add_trace(
        go.Scatter(
            x=wrist_3_pos,
            y=tof_df[f"{tof_name}_filtered_data"],
            mode="markers",
        )
    )

    fig.update_layout(
        title=dict(text=f"{tof_name} MAF readings vs. ur5e__wrist_3 position"),
        xaxis=dict(title="Wrist-3 position"),
        yaxis=dict(title="Distance (m)"),
    )

    fig.show()

    return fig


# def plot_tof_trial(
#     data: pd.DataFrame, topic_name: str, trial_num: int, start_time: float = 0.0, fig: go.Figure = None
# ) -> go.Figure:
#     if fig is None:
#         fig = go.Figure()

#     fig.add_trace(
#         go.Scatter(
#             x=data[f"{topic_name}_ts"] - start_time,
#             y=data[f"{topic_name}_data"],
#             mode="markers",
#             name=f"{topic_name}__{trial_num}",
#         )
#     )
#     fig.update_layout(title=dict(text=f"{topic_name}__{trial_num}"))
#     fig.update_xaxes(title_text="Time (s)")
#     fig.update_yaxes(title_text="Distance (m)")
#     return fig
