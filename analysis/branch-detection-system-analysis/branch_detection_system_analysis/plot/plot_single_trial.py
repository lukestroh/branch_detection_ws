#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates

# from branch_detection_system_analysis.fit import curve_fitting as cf
# import final_approach_controller.curve_fitting as cf
from branch_detection_system_analysis.plot import plotly_helpers as ph
from branch_detection_system_analysis.plot import plotting_backend as pb
import glob
import numpy as np
from numpy.typing import ArrayLike
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
import plotly.subplots
import os
import scipy.optimize as so
import traceback
import pprint as pp

import scipy.signal as ssi

from branch_detection_system_analysis.plot import debug_plots as dplot


ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
warehouse_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse")

# trial_file_name = "bds__prosser_allen_t3.3.3__20250220_16-53-40"
trial_file_name = "bds__arm_farm__20250519_15-27-56"  # NOTE: Excellent edge case
# trial_file_name = "bds__arm_farm__20250519_14-47-40"

# trial_file_name = "bds__arm_farm__20250519_16-01-01" # NOTE: Generic case

def angular_distance(a, b):
    return np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))

def filter_minima_by_angle_proximity(joint_states, distances, valley_idxs, angle_thresh=0.1):
    # Sort valley indices by angle
    sorted_idxs = valley_idxs[np.argsort(joint_states[valley_idxs])]
    filtered_idxs = []
    distances = np.asarray(distances)

    # Group nearby angles
    group = [sorted_idxs[0]]
    for idx in sorted_idxs[1:]:
        prev_idx = group[-1]
        if angular_distance(joint_states[idx], joint_states[prev_idx]) < angle_thresh:
            group.append(idx)
        else:
            # Select the point with the shortest distance in the group
            best_idx = group[np.argmin(distances[group])]
            filtered_idxs.append(best_idx)
            group = [idx]

    # Don't forget the last group
    if group:
        best_idx = group[np.argmin(distances[group])]
        filtered_idxs.append(best_idx)

    if len(filtered_idxs) >= 2:
        first_idx = filtered_idxs[0]
        last_idx = filtered_idxs[-1]
        if angular_distance(joint_states[first_idx], joint_states[last_idx]) < angle_thresh:
            best_idx = [first_idx, last_idx][np.argmin([distances[first_idx], distances[last_idx]])]
            filtered_idxs = [i for i in filtered_idxs if i not in (first_idx, last_idx)]
            filtered_idxs.append(best_idx)

    return np.array(filtered_idxs)


def main():

    def _wrap_angles_to_circle(angles: np.ndarray | float):
        return (angles + np.pi) % (2 * np.pi) - np.pi

    trial_files = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name=trial_file_name)
    df_dict = pb.build_df_dict_from_files(data_dict=None, files=trial_files)

    pp.pprint(df_dict)

    data_dict = {}
    # data_dict['tof0_filtered'] = df_dict['tof0_filtered'].to_dict(orient='list').keys()
    data_dict["tof0_raw"] = df_dict["tof0_raw"].to_dict(orient="list")
    data_dict["tof1_raw"] = df_dict["tof1_raw"].to_dict(orient="list")
    data_dict["tof0_filtered"] = df_dict["tof0_filtered"].to_dict(orient="list")
    data_dict["tof1_filtered"] = df_dict["tof1_filtered"].to_dict(orient="list")
    data_dict["joint_states"] = df_dict["joint_states"].to_dict(orient="list")

    ######################################################################################################3
    # Separated Data
    ##########################
    _tof0_js_ts, joint_states_tof0_data = pb.get_list_rows_at_closest_timestamps(
        data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof0_filtered"]["tof0_filtered_ts"]
    )
    _tof1_js_ts, joint_states_tof1_data = pb.get_list_rows_at_closest_timestamps(
        data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof1_filtered"]["tof1_filtered_ts"]
    )

    sensor_data_dict = {"tof0": {}, "tof1": {}}
    sensor_data_dict["tof0"]["raw_tof_ts"] = data_dict["tof0_raw"]["tof0_raw_ts"]
    sensor_data_dict["tof0"]["raw_tof_data"] = data_dict["tof0_raw"]["tof0_raw_data"]
    sensor_data_dict["tof0"]["tof_ts"] = data_dict["tof0_filtered"]["tof0_filtered_ts"]
    sensor_data_dict["tof0"]["tof_data"] = data_dict["tof0_filtered"]["tof0_filtered_data"]
    sensor_data_dict["tof0"]["joint_states_ts"] = _tof0_js_ts
    sensor_data_dict["tof0"]["joint_states_data"] = joint_states_tof0_data + np.pi / 2
    sensor_data_dict["tof1"]["raw_tof_ts"] = data_dict["tof1_raw"]["tof1_raw_ts"]
    sensor_data_dict["tof1"]["raw_tof_data"] = data_dict["tof1_raw"]["tof1_raw_data"]
    sensor_data_dict["tof1"]["tof_ts"] = data_dict["tof1_filtered"]["tof1_filtered_ts"]
    sensor_data_dict["tof1"]["tof_data"] = data_dict["tof1_filtered"]["tof1_filtered_data"]
    sensor_data_dict["tof1"]["joint_states_ts"] = _tof1_js_ts
    sensor_data_dict["tof1"]["joint_states_data"] = joint_states_tof1_data - np.pi / 2
    ###################################################################################################3
    # Combined Data
    ##########################
    all_data_dict = {}
    all_data_dict["tof_ts"] = sensor_data_dict["tof0"]["tof_ts"] + sensor_data_dict["tof1"]["tof_ts"]
    all_data_dict["tof_data"] = sensor_data_dict["tof0"]["tof_data"] + sensor_data_dict["tof1"]["tof_data"]
    all_data_dict["joint_states_ts"] = np.concatenate(
        (sensor_data_dict["tof0"]["joint_states_ts"], sensor_data_dict["tof1"]["joint_states_ts"])
    )
    all_data_dict["joint_states_data"] = _wrap_angles_to_circle(
        np.concatenate((sensor_data_dict["tof0"]["joint_states_data"], sensor_data_dict["tof1"]["joint_states_data"]))
    )

    # Get minima. We are searching for two
    far_plane_filter = 0.2
    valley_idxs, heights_dict = ssi.find_peaks(
        x=(-1 * np.asarray(all_data_dict["tof_data"])), height=(-1 * far_plane_filter), distance=50
    )

    filtered_valley_idxs = filter_minima_by_angle_proximity(
        joint_states=all_data_dict["joint_states_data"][:,2],
        distances=all_data_dict['tof_data'],
        valley_idxs=valley_idxs,
        angle_thresh=np.radians(30)
    )

    assert len(filtered_valley_idxs) == 2

    wrist3_joint_angles = all_data_dict['joint_states_data'][:,2]
    minimum_angles = wrist3_joint_angles[filtered_valley_idxs]
    

    if np.any(np.isclose(np.pi, np.abs(minimum_angles), atol=0.1)):
        print("close to pi/-pi overlap")
    else:
        angle_midpoint = _wrap_angles_to_circle(np.mean(minimum_angles))

        # Find closest index in joint_states_data to this midpoint
        angular_diffs = np.abs(angular_distance(a=wrist3_joint_angles, b=angle_midpoint))
        mid_idx = np.where(angular_diffs == np.min(angular_diffs))[0][0]
        
        if filtered_valley_idxs[0] < filtered_valley_idxs[1]:
            section1 = wrist3_joint_angles[:mid_idx]
            section2 = wrist3_joint_angles[mid_idx:]
        else:
            section1 = wrist3_joint_angles[mid_idx:]
            section2 = wrist3_joint_angles[:mid_idx]

        return section1, section2, mid_idx
    


    if False:
        parabola_fig = dplot.plot_tof_vs_joint_state(data=sensor_data_dict["tof0"], name="tof0")
        proj_2d_fig = dplot.plot_2d_tof_projection(data=sensor_data_dict["tof0"], name="tof0")
        proj_3d_fig = dplot.plot_3d_tof_projection(data=sensor_data_dict["tof0"], name="tof0")

        parabola_fig = dplot.plot_tof_vs_joint_state(data=sensor_data_dict["tof1"], name="tof1", fig=parabola_fig)
        proj_2d_fig = dplot.plot_2d_tof_projection(data=sensor_data_dict["tof1"], name="tof1", fig=proj_2d_fig)
        proj_3d_fig = dplot.plot_3d_tof_projection(data=sensor_data_dict["tof1"], name="tof1", fig=proj_3d_fig)

        # Add minima to plot
        for i, idx in enumerate(valley_idxs):
            if i == 0:
                showlegend = True
            else:
                showlegend = False
            parabola_fig.add_trace(
                go.Scatter(
                    x=[all_data_dict["joint_states_data"][idx][2]],
                    y=[all_data_dict['tof_data'][idx]],
                    mode='markers',
                    name='minimum',
                    marker=dict(size=20, color="LightSkyBlue"),
                    showlegend=showlegend,
                    legendgroup="minima",
                    legendgrouptitle=dict(text="minima")
                )
            )
        for i, idx in enumerate(filtered_valley_idxs):
            if i == 0:
                showlegend = True
            else:
                showlegend = False
            parabola_fig.add_trace(
                go.Scatter(
                    x=[all_data_dict["joint_states_data"][idx][2]],
                    y=[all_data_dict['tof_data'][idx]],
                    mode='markers',
                    name='filtered_minimum',
                    marker=dict(size=20, color="orange"),
                    showlegend=showlegend,
                    legendgroup="filtered_minima",
                    legendgrouptitle=dict(text="filtered_minima")
                )
            )
        

        parabola_fig.show()
        proj_2d_fig.show()
        proj_3d_fig.show()

    return

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
        # save_fig=True,
        # save_fig_path=
    )

    # pp.pprint(tof0_time_and_dist)

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
