#!/usr/bin/env python3
import glob
import numpy as np
import os
import pandas as pd
import plotly.graph_objects as go


"""
Data collection functions
"""
def get_files_by_trial_name(warehouse_path: str, name: str) -> list[str]:
    files = glob.glob(os.path.join(warehouse_path, name+"_0") + "/*.h5")
    return files


def get_files_by_topic(warehouse_path: str, topic: str):
    return glob.glob(warehouse_path + f"/**/*{topic}*.h5")


"""
Filtering functions
"""
def filter_files_by_trial_number(files: list[str], trial_number: int) -> list[str]:
    return [file for file in files if file.endswith(f"{str(trial_number).zfill(3)}.h5")]




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


"""
Plotting functions
"""
def plot_imu_data(imu_df: pd.DataFrame, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=imu_df["imu_ts"],
            y=imu_df["imu_ax"],
            name='imu_ax'
        )
    )
    fig.add_trace(
        go.Scatter(
            x=imu_df["imu_ts"],
            y=imu_df["imu_ay"],
            name='imu_ay'
        )
    )
    fig.add_trace(
        go.Scatter(
            x=imu_df["imu_ts"],
            y=imu_df["imu_az"],
            name='imu_az'
        )
    )
    return fig


def plot_linear_wrench_data(wrench_df: pd.DataFrame, fig: go.Figure = None) -> go.Figure:
    if fig is None:
        fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=wrench_df["wrench_ts"],
            y=wrench_df['wrench_fx'],
            name='wrench_fx'
        )
    )
    fig.add_trace(
        go.Scatter(
            x=wrench_df["wrench_ts"],
            y=wrench_df['wrench_fy'],
            name='wrench_fy'
        )
    )
    fig.add_trace(
        go.Scatter(
            x=wrench_df["wrench_ts"],
            y=wrench_df['wrench_fz'],
            name='wrench_fz'
        )
    )
    return fig
