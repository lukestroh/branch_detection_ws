#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates

# from branch_detection_system_analysis.fit import curve_fitting as cf
import final_approach_controller.curve_fitting as cf
from branch_detection_system_analysis.plot import plotly_helpers as ph
from branch_detection_system_analysis.plot import plotting_backend as pb
import glob
import numpy as np
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
import plotly.subplots
import os
import scipy.optimize as so
import traceback


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
warehouse_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse")

# trial_file_name = "bds__prosser_allen_t3.3.3__20250220_16-53-40"
trial_file_name = "bds__arm_farm__20250519_15-27-56"
# trial_file_name = "bds__arm_farm__20250519_14-47-40"

trial_file_name = "bds__arm_farm__20250519_16-01-01"


def main():
    trial_files = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name=trial_file_name)
    df_dict = pb.build_df_dict_from_files(data_dict=None, files=trial_files)

    tof0_time_and_dist = cf.get_branch_center_time_and_distance(
        df_dict=df_dict,
        filter_far_plane=0.20,
        sensor_name="tof0",
        debug_plot=True,
        min_samples=10,
        max_trials=20,
        residual_threshold=0.008,
        window_overlap_ratio=9 / 10,
        window_size=2.0,
    )

    # if tof0_time_and_dist is not None:
    #     tof0_branch_center_time, tof0_branch_center_min = tof0_time_and_dist
    # else:
    #     # print(tof0_time_and_dist)
    #     ...

    # fig = pb.plot_tof_vs_joint_state(df_dict=df_dict, tof_name="tof0")

    # fig = pb.plot_tof_vs_joint_state(df_dict=df_dict, tof_name="tof1")

    return


if __name__ == "__main__":
    main()
