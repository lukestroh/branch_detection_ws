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

import pprint as pp


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


# ==========================
#    Filtering functions
# ==========================
def filter_files_by_trial_number(files: list[str], trial_number: int) -> list[str]:
    """WARNING: Only for multi-trial use"""
    return [file for file in files if file.endswith(f"{str(trial_number).zfill(3)}.h5")]


def filter_files_by_topic(files: list[str], topic: str) -> list[str]:
    import pprint as pp

    match = re.fullmatch(r"[A-Za-z0-9_.\-/]+", topic)
    if match is None:
        return []
    else:
        pattern = rf"__{re.escape(topic)}__(?=)"
        return [f for f in files if re.search(pattern, f)]


def filter_files_by_topics(files: list[str], topics: list[str]) -> list[str]:
    _files = []
    for topic in topics:
        _files.extend(filter_files_by_topic(files, topic))
    return _files


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
        "topic": topic_match.group(1) if topic_match else None
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
            datetime_str = datetime_match.group(0).strip('_')
            topic_str = topic_match.group(1)
            grouped_by_datetime_by_topic[datetime_str][topic_str].append(file)
    return group_files_by_datetime_by_topic


# =================================
#    Data organization functions
# =================================
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


def get_df_rows_at_closest_timestamp(df_dict: dict, topic_name: str, timestamps: ArrayLike) -> pd.DataFrame:
    # """Gets the closest set of TF frames at a given timestamp"""
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


# ==========================
#    Plotting functions
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


"""
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
"""