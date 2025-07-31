#!/usr/bin/env python3
from branch_detection_system_analysis.bag_reader.ros_constants import TransitionStates

# from branch_detection_system_analysis.fit import curve_fitting as cf
# import final_approach_controller.curve_fitting as cf
from branch_detection_system_analysis.fit import branch_processing as bp
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

import final_approach_controller.curve_fitting as cf
import final_approach_controller.data_processing as dp


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


def main1():
    from branch_detection_system_analysis.node import dummy_node

    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse")

    sunlight_trial_file_name = "bds__prosser_allen_t1.1.4__20250220_14-26-43"

    # trial_file_name = "bds__prosser_allen_t3.3.3__20250220_16-53-40"
    trial_file_name = "bds__arm_farm__20250519_15-27-56"  # NOTE: Excellent edge case
    # trial_file_name = "bds__arm_farm__20250519_14-47-40"

    # trial_file_name = "bds__arm_farm__20250519_16-01-01" # NOTE: Generic case

    # trial_file_name = "bds__arm_farm__20250722_22-32-22"
    # trial_file_name = "bds__arm_farm__20250724_17-07-53"

    # NEW ONES TO TEST
    trial_file_name = "bds__arm_farm__20250727_18-34-28"
    # trial_file_name = "bds__arm_farm__20250727_18-35-54"
    # trial_file_name = "bds__arm_farm__20250727_18-48-51"

    node = dummy_node.DummyNode()

    def _wrap_angles_to_circle(angles: np.ndarray | float):
        return (angles + np.pi) % (2 * np.pi) - np.pi

    trial_files = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name=trial_file_name)
    df_dict = pb.build_df_dict_from_files(data_dict=None, files=trial_files)

    # activate_df = pb.filter_transition_events_for_controller_active(df=df_dict['fpc_transition_events'])
    # activate_fpc_ts = activate_df.at[0,'controller_transition_events_ts']

    data_dict = {}
    # data_dict['tof0_filtered'] = df_dict['tof0_filtered'].to_dict(orient='list').keys()
    data_dict["tof0_raw"] = df_dict["tof0_raw"].to_dict(orient="list")
    data_dict["tof1_raw"] = df_dict["tof1_raw"].to_dict(orient="list")
    data_dict["tof0_filtered"] = df_dict["tof0_filtered"].to_dict(orient="list")
    data_dict["tof1_filtered"] = df_dict["tof1_filtered"].to_dict(orient="list")
    data_dict["joint_states"] = df_dict["joint_states"].to_dict(orient="list")
    data_dict["tf"] = df_dict["tf"]
    data_dict["tf_static"] = df_dict["tf_static"]

    print(len(data_dict["tof0_filtered"]["tof0_filtered_data"]))

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
    sensor_data_dict["tof0"]["sensor_id"] = [0] * len(sensor_data_dict["tof0"]["tof_ts"])

    sensor_data_dict["tof1"]["raw_tof_ts"] = data_dict["tof1_raw"]["tof1_raw_ts"]
    sensor_data_dict["tof1"]["raw_tof_data"] = data_dict["tof1_raw"]["tof1_raw_data"]
    sensor_data_dict["tof1"]["tof_ts"] = data_dict["tof1_filtered"]["tof1_filtered_ts"]
    sensor_data_dict["tof1"]["tof_data"] = data_dict["tof1_filtered"]["tof1_filtered_data"]
    sensor_data_dict["tof1"]["joint_states_ts"] = _tof1_js_ts
    sensor_data_dict["tof1"]["joint_states_data"] = joint_states_tof1_data - np.pi / 2
    sensor_data_dict["tof1"]["sensor_id"] = [1] * len(sensor_data_dict["tof1"]["tof_ts"])

    ###########################
    # Combined Data
    ##########################
    all_data_dict = {}
    all_data_dict["raw_tof_ts"] = np.concatenate(
        [sensor_data_dict["tof0"]["raw_tof_ts"], sensor_data_dict["tof1"]["raw_tof_ts"]]
    )
    all_data_dict["raw_tof_data"] = np.concatenate(
        [sensor_data_dict["tof0"]["raw_tof_data"], sensor_data_dict["tof1"]["raw_tof_data"]]
    )
    all_data_dict["tof_ts"] = np.concatenate([sensor_data_dict["tof0"]["tof_ts"], sensor_data_dict["tof1"]["tof_ts"]])
    all_data_dict["tof_data"] = np.concatenate(
        [sensor_data_dict["tof0"]["tof_data"], sensor_data_dict["tof1"]["tof_data"]]
    )
    all_data_dict["joint_states_ts"] = np.concatenate(
        [sensor_data_dict["tof0"]["joint_states_ts"], sensor_data_dict["tof1"]["joint_states_ts"]]
    )
    all_data_dict["joint_states_data"] = np.concatenate(
        (sensor_data_dict["tof0"]["joint_states_data"], sensor_data_dict["tof1"]["joint_states_data"])
    )
    all_data_dict["sensor_id"] = np.concatenate(
        (sensor_data_dict["tof0"]["sensor_id"], sensor_data_dict["tof1"]["sensor_id"])
    )

    # Slice where the trial starts
    rotation_started_ts = df_dict["rotation_started"].at[0, "rotation_event_ts"]
    rotation_stopped_ts = df_dict["rotation_stopped"].at[0, "rotation_event_ts"]

    trial_idxs = np.where(
        (all_data_dict["tof_ts"] > rotation_started_ts) & (all_data_dict["tof_ts"] < rotation_stopped_ts)
    )
    all_data_dict["raw_tof_ts"] = all_data_dict["raw_tof_ts"][trial_idxs]
    all_data_dict["raw_tof_data"] = all_data_dict["raw_tof_data"][trial_idxs]
    all_data_dict["tof_ts"] = all_data_dict["tof_ts"][trial_idxs]
    all_data_dict["tof_data"] = all_data_dict["tof_data"][trial_idxs]
    all_data_dict["joint_states_ts"] = all_data_dict["joint_states_ts"][trial_idxs]
    all_data_dict["joint_states_data"] = all_data_dict["joint_states_data"][trial_idxs]
    all_data_dict["sensor_id"] = all_data_dict["sensor_id"][trial_idxs]

    # Sort all data by wrist 3 joint state
    sorted_indices = np.argsort(all_data_dict["joint_states_data"][:, 2])
    all_data_dict["raw_tof_ts"] = all_data_dict["raw_tof_ts"][sorted_indices]
    all_data_dict["raw_tof_data"] = all_data_dict["raw_tof_data"][sorted_indices]
    all_data_dict["tof_ts"] = all_data_dict["tof_ts"][sorted_indices]
    all_data_dict["tof_data"] = all_data_dict["tof_data"][sorted_indices]
    all_data_dict["joint_states_ts"] = all_data_dict["joint_states_ts"][sorted_indices]
    all_data_dict["joint_states_data"] = all_data_dict["joint_states_data"][sorted_indices]
    all_data_dict["sensor_id"] = all_data_dict["sensor_id"][sorted_indices]

    separated_data_dict = dp.separate_tof_data_by_curve(node=node, all_data_dict=all_data_dict, save_fig=False)

    separated_data_dict["s0"]["joint_states_data"][:, 2] = dp.amend_joint_angle_discontinuity(
        node=node,
        joint_angles=separated_data_dict["s0"]["joint_states_data"][:, 2],
        indices=separated_data_dict["s0"]["indices"],
    )

    # print(separated_data_dict['s0'])
    # return
    time_and_center_res_dict = {}
    for section_name, section in separated_data_dict.items():
        time_and_center_res_dict[section_name] = {}
        sec_time_and_dist = cf.get_branch_center_time_and_distance(
            data=section,
            far_plane_filter=node._param_far_plane_filter,
            section_name=section_name,
            show_fig=False,
            debug_plot=True,
            save_fig=False,
            # save_fig_path=self.bag_record_path,
            window_size=0.4,
            window_overlap_ratio=7 / 10,
            min_samples=10,
            max_trials=20,
            residual_threshold=0.008,
            node=node,
        )

        if sec_time_and_dist is not None:
            timestamp, dist, sensor_id = sec_time_and_dist
            time_and_center_res_dict[section_name]["time"] = timestamp
            time_and_center_res_dict[section_name]["min_dist"] = dist
            time_and_center_res_dict[section_name]["sensor_id"] = sensor_id

    branch_center_point, branch_vec_normalized, tof0_vec_base_frame, tof1_vec_base_frame = bp.get_branch_vec_from_tof(
        node=node, data_dict=data_dict, time_and_center_res_dict=time_and_center_res_dict, return_frames=True
    )

    timestamp_tool0 = all_data_dict["tof_ts"][-1]
    desired_eef_xyz = bp.get_desired_position_from_branch_vec(
        node=node,
        branch_center_point=branch_center_point,
        branch_vec=branch_vec_normalized,
        time=timestamp_tool0,
        data_dict=data_dict,
    )

    desired_orientation_quat, desired_orientation_vec = bp.get_desired_orientation_from_branch_vec(
        branch_center_point=branch_center_point,
        branch_vec=branch_vec_normalized,
        desired_eef_xyz=desired_eef_xyz,
    )

    fig = dplot.plot_branch_projection(
        tof0=tof0_vec_base_frame,
        tof1=tof1_vec_base_frame,
        branch_center_pos=branch_center_point[:3],
        branch_vec_ori=branch_vec_normalized,
        base_origin_name=node._param_robot_base_part,
        desired_eef_pos=desired_eef_xyz,
        desired_eef_ori=desired_orientation_vec,
        save_fig=False,
        # save_fig_dir=self.bag_record_path,
    )
    fig.show()

    # pp.pprint(tof0_time_and_dist)

    # if tof0_time_and_dist is not None:
    #     tof0_branch_center_time, tof0_branch_center_min = tof0_time_and_dist
    # else:
    #     # print(tof0_time_and_dist)
    #     ...

    # fig = pb.plot_tof_vs_joint_state(df_dict=df_dict, tof_name="tof0")

    # fig = pb.plot_tof_vs_joint_state(df_dict=df_dict, tof_name="tof1")

    return


def main():
    trial_files = pb.get_files_by_trial_name(warehouse_path=warehouse_path, name=sunlight_trial_file_name)
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
    # _tof0_js_ts, joint_states_tof0_data = pb.get_list_rows_at_closest_timestamps(
    #     data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof0_filtered"]["tof0_filtered_ts"]
    # )
    # _tof1_js_ts, joint_states_tof1_data = pb.get_list_rows_at_closest_timestamps(
    #     data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof1_filtered"]["tof1_filtered_ts"]
    # )
    _tof0_js_ts, joint_states_raw_tof0_data = pb.get_list_rows_at_closest_timestamps(
        data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof0_raw"]["tof0_raw_ts"]
    )
    _tof1_js_ts, joint_states_raw_tof1_data = pb.get_list_rows_at_closest_timestamps(
        data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof1_raw"]["tof1_raw_ts"]
    )
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
    # sensor_data_dict["tof0"]["joint_states_ts"] = _tof0_js_ts
    sensor_data_dict["tof0"]["joint_states_raw_data"] = joint_states_raw_tof0_data + np.pi / 2
    sensor_data_dict["tof0"]["joint_states_data"] = joint_states_tof0_data + np.pi / 2

    sensor_data_dict["tof1"]["raw_tof_ts"] = data_dict["tof1_raw"]["tof1_raw_ts"]
    sensor_data_dict["tof1"]["raw_tof_data"] = data_dict["tof1_raw"]["tof1_raw_data"]
    sensor_data_dict["tof1"]["tof_ts"] = data_dict["tof1_filtered"]["tof1_filtered_ts"]
    sensor_data_dict["tof1"]["tof_data"] = data_dict["tof1_filtered"]["tof1_filtered_data"]
    # sensor_data_dict["tof1"]["joint_states_ts"] = _tof1_js_ts
    sensor_data_dict["tof1"]["joint_states_raw_data"] = joint_states_raw_tof1_data - np.pi / 2
    sensor_data_dict["tof1"]["joint_states_data"] = joint_states_tof1_data - np.pi / 2

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=sensor_data_dict["tof0"]["joint_states_raw_data"][:, 2],
            y=sensor_data_dict["tof0"]["raw_tof_data"],
            name="tof0_raw",
            mode="markers",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=sensor_data_dict["tof1"]["joint_states_raw_data"][:, 2],
            y=sensor_data_dict["tof1"]["raw_tof_data"],
            name="tof1_raw",
            mode="markers",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=sensor_data_dict["tof0"]["joint_states_data"][:, 2],
            y=sensor_data_dict["tof0"]["tof_data"],
            name="tof0_filtered",
            mode="lines",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=sensor_data_dict["tof1"]["joint_states_data"][:, 2],
            y=sensor_data_dict["tof1"]["tof_data"],
            name="tof1_filtered",
            mode="lines",
        )
    )

    fig.show()

    # pp.pprint(sensor_data_dict)
    print(sensor_data_dict)
    return


if __name__ == "__main__":
    main1()
