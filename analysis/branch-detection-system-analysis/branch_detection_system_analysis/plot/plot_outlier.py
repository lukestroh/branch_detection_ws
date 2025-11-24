#!/usr/bin/env python3
import plotly.graph_objects as go
import numpy as np
import pandas as pd

import h5py as hp

files = [
    "/home/luke/branch_detection_ws/bags/2025_ToFBranchDetection/warehouse/bds__prosser_roza_t1.1.2__20250221_09-57-31_0/bds__prosser_roza_t1.1.2__20250221_09-57-31_0__tf__014.h5",
    "/home/luke/branch_detection_ws/bags/2025_ToFBranchDetection/warehouse/bds__prosser_roza_t1.1.2__20250221_09-57-31_0/bds__prosser_roza_t1.1.2__20250221_09-57-31_0__tof0_raw__014.h5",
    "/home/luke/branch_detection_ws/bags/2025_ToFBranchDetection/warehouse/bds__prosser_roza_t1.1.2__20250221_09-57-31_0/bds__prosser_roza_t1.1.2__20250221_09-57-31_0__tof1_raw__014.h5",
    "/home/luke/branch_detection_ws/bags/2025_ToFBranchDetection/warehouse/bds__prosser_roza_t1.1.2__20250221_09-57-31_0/bds__prosser_roza_t1.1.2__20250221_09-57-31_0__tof0_filtered__014.h5",
    "/home/luke/branch_detection_ws/bags/2025_ToFBranchDetection/warehouse/bds__prosser_roza_t1.1.2__20250221_09-57-31_0/bds__prosser_roza_t1.1.2__20250221_09-57-31_0__tof1_filtered__014.h5",
]

tof1_filtered = pd.read_hdf(path_or_buf="/home/luke/branch_detection_ws/bags/2025_ToFBranchDetection/warehouse/bds__prosser_roza_t1.1.2__20250221_09-57-31_0/bds__prosser_roza_t1.1.2__20250221_09-57-31_0__tof1_filtered__014.h5")

print(tof1_filtered)

fig = go.Figure()
fig.add_trace(
    go.Scatter(
        x=tof1_filtered['tof1_filtered_ts'],
        y=tof1_filtered['tof1_filtered_data'],
        mode="markers",
    )
)
fig.show()
