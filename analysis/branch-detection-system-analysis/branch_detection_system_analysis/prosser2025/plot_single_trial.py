#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates
from branch_detection_system_analysis.prosser2025 import curve_fitting as cf
from branch_detection_system_analysis.prosser2025 import plotly_helpers as ph
from branch_detection_system_analysis.prosser2025 import plotting_backend as pb
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


def filter_imu_data(imu_df: pd.DataFrame) -> pd.DataFrame:
    fft_res = fft(imu_df['imu_ax'])
    size = imu_df['imu_ax'].size

    freq = fftfreq(n=size, d=1/833)

    print(len(fft_res))


    return fft_res, freq


def main():
    trial_files = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name="bds__arm_farm__20250505_11-28-09")
    df_dict = pb.build_df_dict_from_files(data_dict=None, files=trial_files)

    fft_res, freq = filter_imu_data(imu_df=df_dict['imu'])

    fig = pb.plot_imu_data(imu_df=df_dict["imu"])
    fig = pb.plot_linear_wrench_data(wrench_df=df_dict['wrench'], fig=fig)
    fig.show()
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
