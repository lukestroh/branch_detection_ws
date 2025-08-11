#!/usr/bin/env python3
from branch_detection_system_analysis.fit import branch_processing as bp
from branch_detection_system_analysis.node import dummy_node
from branch_detection_system_analysis.plot import plotting_backend as pb
from final_approach_controller import curve_fitting as cf
from final_approach_controller import data_processing as dp
import numpy as np
import os
import pandas as pd
import plotly.graph_objects as go
import pprint as pp

from final_approach_controller_msgs.action import GenerateCylindricalPoses
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation

import traceback

import datetime as dt

from collections import defaultdict


def get_files():
    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))

    # all_files = {"tf": [], "tf_static": [], "start_position": [], "rotation_started": [], "tof0_filtered": [], "tof1_filtered": [], }
    all_files = []

    # 07 - 29
    files_by_date = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250729")
    files_tf = pb.filter_files_by_topic(files=files_by_date, topic="tf")
    files_tf_static = pb.filter_files_by_topic(files=files_by_date, topic="tf_static")
    files_start_position = pb.filter_files_by_topic(files=files_by_date, topic="trial_start_pose")
    # files_start_position_idx = pb.filter_files_by_topic(files=files_by_date, topic='trial_start_pose_index')
    files_joint_states = pb.filter_files_by_topic(files=files_by_date, topic="joint_states")
    files_rotation_started = pb.filter_files_by_topic(files=files_by_date, topic="rotation_started")
    files_rotation_stopped = pb.filter_files_by_topic(files=files_by_date, topic="rotation_stopped")
    files_tof0_filtered = pb.filter_files_by_topic(files=files_by_date, topic="tof0_filtered")
    files_tof1_filtered = pb.filter_files_by_topic(files=files_by_date, topic="tof1_filtered")
    _idx = 26
    files_tf = files_tf[_idx:]
    files_tf_static = files_tf_static[_idx:]
    files_start_position = files_start_position[_idx:]
    # files_start_position_idx = files_start_position_idx[_idx:]
    # files_joint_states = files_joint_states[_idx:]
    files_rotation_started = files_rotation_started[_idx:]
    files_tof0_filtered = files_tof0_filtered[_idx:]
    files_tof1_filtered = files_tof1_filtered[_idx:]

    all_files += (
        files_tf
        + files_tf_static
        + files_start_position
        + files_rotation_started
        + files_rotation_stopped
        + files_tof0_filtered
        + files_tof1_filtered
        + files_joint_states
    )
    # all_files['tf'] += files_tf
    # all_files['tf_static'] += files_tf_static
    # all_files['start_position'] += files_start_position
    # all_files['rotation_started'] += files_rotation_started
    # all_files['tof0_filtered'] += files_tof0_filtered
    # all_files['tof1_filtered'] += files_tof1_filtered

    files_by_date = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250730")
    files_tf = pb.filter_files_by_topic(files=files_by_date, topic="tf")
    files_tf_static = pb.filter_files_by_topic(files=files_by_date, topic="tf_static")
    files_start_position = pb.filter_files_by_topic(files=files_by_date, topic="trial_start_pose")
    # files_start_position_idx = pb.filter_files_by_topic(files=files_by_date, topic='trial_start_pose_index')
    files_joint_states = pb.filter_files_by_topic(files=files_by_date, topic="joint_states")
    files_rotation_started = pb.filter_files_by_topic(files=files_by_date, topic="rotation_started")
    files_rotation_stopped = pb.filter_files_by_topic(files=files_by_date, topic="rotation_stopped")
    files_tof0_filtered = pb.filter_files_by_topic(files=files_by_date, topic="tof0_filtered")
    files_tof1_filtered = pb.filter_files_by_topic(files=files_by_date, topic="tof1_filtered")
    _idx = -6
    files_tf = files_tf[:_idx]
    files_tf_static = files_tf_static[:_idx]
    files_start_position = files_start_position[:_idx]
    # files_start_position_idx = files_start_position_idx[:_idx]
    files_joint_states = files_joint_states[:_idx]
    files_rotation_started = files_rotation_started[:_idx]
    files_tof0_filtered = files_tof0_filtered[:_idx]
    files_tof1_filtered = files_tof1_filtered[:_idx]

    all_files += (
        files_tf
        + files_tf_static
        + files_start_position
        + files_rotation_started
        + files_rotation_stopped
        + files_tof0_filtered
        + files_tof1_filtered
        + files_joint_states
    )

    # all_files['tf'] += files_tf
    # all_files['tf_static'] += files_tf_static
    # all_files['start_position'] += files_start_position
    # all_files['rotation_started'] += files_rotation_started
    # all_files['tof0_filtered'] += files_tof0_filtered
    # all_files['tof1_filtered'] += files_tof1_filtered

    grouped_files = pb.group_files_by_datetime_by_topic(files=all_files)

    return grouped_files


def get_projected_tof_readings(grouped_files: defaultdict):
    node = dummy_node.DummyNode()

    tof_readings_pts = []

    for trial_name, trial_data in grouped_files.items():
        df_dict = {}
        for topic_name, data_file_path in trial_data.items():
            df_dict[topic_name] = pd.read_hdf(data_file_path[0])

        data_dict = {}
        for topic_name, data_df in df_dict.items():
            data_dict[topic_name] = data_df.to_dict(orient="list")

        try:
            data_dict["trial_start_pose"]
        except KeyError:
            print(trial_name)
            continue
        # print(data_dict["trial_start_pose_index"])
        # import sys
        # sys.exit()

        _tof0_js_ts, joint_states_tof0_data = pb.get_list_rows_at_closest_timestamps(
            data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof0_filtered"]["tof0_filtered_ts"]
        )
        _tof1_js_ts, joint_states_tof1_data = pb.get_list_rows_at_closest_timestamps(
            data_dict=data_dict, topic_name="joint_states", timestamps=data_dict["tof1_filtered"]["tof1_filtered_ts"]
        )

        ##########################
        # Sensor Data
        ##########################
        sensor_data_dict = {"tof0": {}, "tof1": {}}
        # sensor_data_dict["tof0"]["raw_tof_ts"] = data_dict["tof0_raw"]["tof0_raw_ts"]
        # sensor_data_dict["tof0"]["raw_tof_data"] = data_dict["tof0_raw"]["tof0_raw_data"]
        sensor_data_dict["tof0"]["tof_ts"] = data_dict["tof0_filtered"]["tof0_filtered_ts"]
        sensor_data_dict["tof0"]["tof_data"] = data_dict["tof0_filtered"]["tof0_filtered_data"]
        sensor_data_dict["tof0"]["joint_states_ts"] = _tof0_js_ts
        sensor_data_dict["tof0"]["joint_states_data"] = joint_states_tof0_data + np.pi / 2
        sensor_data_dict["tof0"]["sensor_id"] = [0] * len(sensor_data_dict["tof0"]["tof_ts"])

        # sensor_data_dict["tof1"]["raw_tof_ts"] = data_dict["tof1_raw"]["tof1_raw_ts"]
        # sensor_data_dict["tof1"]["raw_tof_data"] = data_dict["tof1_raw"]["tof1_raw_data"]
        sensor_data_dict["tof1"]["tof_ts"] = data_dict["tof1_filtered"]["tof1_filtered_ts"]
        sensor_data_dict["tof1"]["tof_data"] = data_dict["tof1_filtered"]["tof1_filtered_data"]
        sensor_data_dict["tof1"]["joint_states_ts"] = _tof1_js_ts
        sensor_data_dict["tof1"]["joint_states_data"] = joint_states_tof1_data - np.pi / 2
        sensor_data_dict["tof1"]["sensor_id"] = [1] * len(sensor_data_dict["tof1"]["tof_ts"])

        ###########################
        # Combined Data
        ###########################
        all_data_dict = {}
        # all_data_dict["raw_tof_ts"] = np.concatenate(
        #     [sensor_data_dict["tof0"]["raw_tof_ts"], sensor_data_dict["tof1"]["raw_tof_ts"]]
        # )
        # all_data_dict["raw_tof_data"] = np.concatenate(
        #     [sensor_data_dict["tof0"]["raw_tof_data"], sensor_data_dict["tof1"]["raw_tof_data"]]
        # )
        all_data_dict["tof_ts"] = np.concatenate(
            [sensor_data_dict["tof0"]["tof_ts"], sensor_data_dict["tof1"]["tof_ts"]]
        )
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
        try:
            rotation_started_ts = df_dict["rotation_started"].at[0, "rotation_event_ts"]
            rotation_stopped_ts = df_dict["rotation_stopped"].at[0, "rotation_event_ts"]
        except KeyError:
            continue

        trial_idxs = np.where(
            (all_data_dict["tof_ts"] > rotation_started_ts) & (all_data_dict["tof_ts"] < rotation_stopped_ts)
        )
        # all_data_dict["raw_tof_ts"] = all_data_dict["raw_tof_ts"][trial_idxs]
        # all_data_dict["raw_tof_data"] = all_data_dict["raw_tof_data"][trial_idxs]
        all_data_dict["tof_ts"] = all_data_dict["tof_ts"][trial_idxs]
        all_data_dict["tof_data"] = all_data_dict["tof_data"][trial_idxs]
        all_data_dict["joint_states_ts"] = all_data_dict["joint_states_ts"][trial_idxs]
        all_data_dict["joint_states_data"] = all_data_dict["joint_states_data"][trial_idxs]
        all_data_dict["sensor_id"] = all_data_dict["sensor_id"][trial_idxs]

        # Sort all data by wrist 3 joint state
        sorted_indices = np.argsort(all_data_dict["joint_states_data"][:, 2])
        # all_data_dict["raw_tof_ts"] = all_data_dict["raw_tof_ts"][sorted_indices]
        # all_data_dict["raw_tof_data"] = all_data_dict["raw_tof_data"][sorted_indices]
        all_data_dict["tof_ts"] = all_data_dict["tof_ts"][sorted_indices]
        all_data_dict["tof_data"] = all_data_dict["tof_data"][sorted_indices]
        all_data_dict["joint_states_ts"] = all_data_dict["joint_states_ts"][sorted_indices]
        all_data_dict["joint_states_data"] = all_data_dict["joint_states_data"][sorted_indices]
        all_data_dict["sensor_id"] = all_data_dict["sensor_id"][sorted_indices]

        

        if all_data_dict["tof_data"].size == 0:
            print("SKIPPING, EMPTY ARRAY")
            continue

        separated_data_dict = dp.separate_tof_data_by_curve(
            node=node, all_data_dict=all_data_dict, save_fig=False, show_fig=False
        )

        

        try:
            if (
                separated_data_dict["s0"]["joint_states_data"][:, 2][0]
                < separated_data_dict["s1"]["joint_states_data"][:, 2][0]
            ):
                separated_data_dict["s0"]["joint_states_data"][:, 2] = dp.amend_joint_angle_discontinuity(
                    node=node,
                    joint_angles=separated_data_dict["s0"]["joint_states_data"][:, 2],
                    indices=separated_data_dict["s0"]["indices"],
                )
            else:
                separated_data_dict["s1"]["joint_states_data"][:, 2] = dp.amend_joint_angle_discontinuity(
                    node=node,
                    joint_angles=separated_data_dict["s1"]["joint_states_data"][:, 2],
                    indices=separated_data_dict["s1"]["indices"],
                )
        except TypeError:
            continue

        break_flag = False
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
            else:
                break_flag = True
                break

        if break_flag:
            continue

        # branch_center_point, branch_vec_normalized, tof0_vec_base_frame, tof1_vec_base_frame = bp.get_branch_vec_from_tof(
        #     node=node, data_dict=data_dict, time_and_center_res_dict=time_and_center_res_dict, return_frames=True
        # )

        # timestamp_tool0 = all_data_dict["tof_ts"][-1]
        # desired_eef_xyz = bp.get_desired_position_from_branch_vec(
        #     node=node,
        #     branch_center_point=branch_center_point,
        #     branch_vec=branch_vec_normalized,
        #     time=timestamp_tool0,
        #     data_dict=data_dict,
        # )

        # desired_orientation_quat, desired_orientation_vec = bp.get_desired_orientation_from_branch_vec(
        #     branch_center_point=branch_center_point,
        #     branch_vec=branch_vec_normalized,
        #     desired_eef_xyz=desired_eef_xyz,
        # )

        # Project tof readings in base frame
        # A_vec_base_frame = bp.get_tof_vec_base_frame(
        #     node=node, data_dict=df_dict, time_and_center_res_dict=time_and_center_res_dict, section_name="s0"
        # )
        # B_vec_base_frame = bp.get_tof_vec_base_frame(
        #     node=node, data_dict=df_dict, time_and_center_res_dict=time_and_center_res_dict, section_name="s1"
        # )

        # mat_A = mat_B = np.identity(4)
        # mat_A[3, :] = A_vec_base_frame
        # mat_B[3, :] = B_vec_base_frame

        branch_center_point, branch_vec_normalized, tofA_reading_vec_base_frame, tofB_reading_vec_base_frame = (
            bp.get_branch_vec_from_tof(
                node=node, data_dict=df_dict, time_and_center_res_dict=time_and_center_res_dict, return_frames=True
            )
        )

        tof_readings_pts.extend([tofA_reading_vec_base_frame, tofB_reading_vec_base_frame])

    return tof_readings_pts


def main():

    files: defaultdict = get_files()

    tof_readings_pts = np.asarray(get_projected_tof_readings(grouped_files=files))

    # Fit
    centroid, direction = cf.fit_3d_linear_pca(points=tof_readings_pts)
    t_vals, coefs = cf.fit_3d_quadratic(points=tof_readings_pts, centroid=centroid, direction=direction)

    quadratic_t_vals, quadratic_projected_points = cf.project_points_onto_curve(
        points=tof_readings_pts, t_vals=t_vals, coefs=coefs
    )

    # Tests:
    residuals = tof_readings_pts[:, 0:3] - quadratic_projected_points
    # print("quadratic residuals:\n", residuals)
    print("quadratic RESIDUALS mean ", np.mean(np.linalg.norm(residuals, axis=1)))
    print("quadratic var:", np.var(residuals))
    print("quadratic std: ", np.std(residuals))

    fig = go.Figure()
    fig.add_trace(
        go.Scatter3d(
            x=tof_readings_pts[:,0],
            y=tof_readings_pts[:,1],
            z=tof_readings_pts[:,2],
            mode='markers',
            marker=dict(size=4)
        )
    )
    fig = pb.plot_quadratic_fit(t_vals=quadratic_t_vals, coefs=coefs, fig=fig)
    fig = pb.plot_quadratic_residuals(points=tof_readings_pts, projected_points=quadratic_projected_points, fig=fig)

    fig.update_layout(scene=dict(aspectmode='data'))
    fig.show()



    return


if __name__ == "__main__":
    main()
