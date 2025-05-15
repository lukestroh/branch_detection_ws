#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates
from branch_detection_system_analysis.fit import curve_fitting as cf
from branch_detection_system_analysis.plot import plotly_helpers as ph
from branch_detection_system_analysis.plot import plotting_backend as pb
import glob
import numpy as np
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
import plotly.subplots
import os
from scipy.fft import fft, fftfreq
from scipy.signal import iirnotch
from scipy.spatial.transform import Rotation
import scipy.optimize as so
import traceback


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
warehouse_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse")

trial_file_name = "bds__prosser_allen_t3.3.3__20250220_16-53-40"


def main():
    trial_files = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name=trial_file_name)
    df_dict = pb.build_df_dict_from_files(data_dict=None, files=trial_files)

    tof0_time_and_dist = cf.get_branch_center_time_and_distance(
        filter_far_plane=0.20,
        raw_ts=df_dict["tof0_raw"]["tof0_raw_ts"],
        raw_data=df_dict["tof0_raw"]["tof0_raw_data"],
        mav_filter_ts=df_dict["tof0_filtered"]["tof0_filtered_ts"],
        mav_filter_data=df_dict["tof0_filtered"]["tof0_filtered_data"],
        sensor_name="tof0",
        debug_plot=True,
        min_samples=10,
        max_trials=20,
        residual_threshold=0.008,
        window_overlap_ratio=2 / 3,
    )

    if tof0_time_and_dist is not None:
        tof0_branch_center_time, tof0_branch_center_min = tof0_time_and_dist
    else:
        # print(tof0_time_and_dist)
        ...

    # print(tof0_time_and_dist)
    # fig = pb.plot_imu_data(imu_df=df_dict["imu"])
    # fig = pb.plot_linear_wrench_data(wrench_df=df_dict['wrench'], fig=fig)
    # fig.show()
    # fig = go.Figure()
    # fig.add_trace(
    #     go.Scatter(
    #         x=freq[:len(freq)],
    #         y=np.abs(fft_res[:len(fft_res)])
    #     )
    # )
    # fig.show()
    return


if __name__ == "__main__":
    main()
