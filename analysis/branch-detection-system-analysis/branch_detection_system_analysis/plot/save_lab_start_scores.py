#!/usr/bin/env python3
from branch_detection_system_analysis.fit import branch_processing as bp
from branch_detection_system_analysis.node import dummy_node
from branch_detection_system_analysis.plot import plotting_backend as pb
from branch_detection_system_analysis.plot import plotly_helpers as ph
from branch_detection_system_analysis.plot import ray_tracing as rt
from final_approach_controller import curve_fitting as cf
from final_approach_controller import data_processing as dp
import numpy as np
import os
import pandas as pd
import plotly.graph_objects as go
import pprint as pp
import h5py as hpy

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
    files_fbrw_localized = pb.filter_files_by_topic(files=files_by_date, topic="fbrw_controller_localization_success")
    files_fbrw_aligned = pb.filter_files_by_topic(files=files_by_date, topic="fbrw_controller_alignment_success")

    _idx = 26
    files_tf = files_tf[_idx:]
    files_tf_static = files_tf_static[_idx:]
    files_start_position = files_start_position[_idx:]

    # files_start_position_idx = files_start_position_idx[_idx:]
    files_joint_states = files_joint_states[_idx:]
    files_rotation_started = files_rotation_started[_idx:]
    files_rotation_stopped = files_rotation_stopped[_idx:]
    files_tof0_filtered = files_tof0_filtered[_idx:]
    files_tof1_filtered = files_tof1_filtered[_idx:]
    files_fbrw_localized = files_fbrw_localized[_idx:]
    files_fbrw_aligned = files_fbrw_aligned[_idx:]

    all_files += (
        files_tf
        + files_tf_static
        + files_start_position
        + files_rotation_started
        + files_rotation_stopped
        + files_tof0_filtered
        + files_tof1_filtered
        + files_joint_states
        + files_fbrw_localized
        + files_fbrw_aligned
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
    files_fbrw_localized = pb.filter_files_by_topic(files=files_by_date, topic="fbrw_controller_localization_success")
    files_fbrw_aligned = pb.filter_files_by_topic(files=files_by_date, topic="fbrw_controller_alignment_success")

    _idx = -6
    files_tf = files_tf[:_idx]
    files_tf_static = files_tf_static[:_idx]
    files_start_position = files_start_position[:_idx]
    # files_start_position_idx = files_start_position_idx[:_idx]
    files_joint_states = files_joint_states[:_idx]
    files_rotation_started = files_rotation_started[:_idx]
    files_rotation_stopped = files_rotation_stopped[:_idx]
    files_tof0_filtered = files_tof0_filtered[:_idx]
    files_tof1_filtered = files_tof1_filtered[:_idx]
    files_fbrw_localized = files_fbrw_localized[:_idx]
    files_fbrw_aligned = files_fbrw_aligned[:_idx]

    all_files += (
        files_tf
        + files_tf_static
        + files_start_position
        + files_rotation_started
        + files_rotation_stopped
        + files_tof0_filtered
        + files_tof1_filtered
        + files_joint_states
        + files_fbrw_localized
        + files_fbrw_aligned
    )

    # all_files['tf'] += files_tf
    # all_files['tf_static'] += files_tf_static
    # all_files['start_position'] += files_start_position
    # all_files['rotation_started'] += files_rotation_started
    # all_files['tof0_filtered'] += files_tof0_filtered
    # all_files['tof1_filtered'] += files_tof1_filtered

    grouped_files = pb.group_files_by_datetime_by_topic(files=all_files)

    # pp.pprint(grouped_files["20250729_21-54-35"])

    return grouped_files


def get_projected_tof_readings(grouped_files: defaultdict):
    node = dummy_node.DummyNode()

    # fbrw_localization = {}
    fbrw_localized = []
    # fbrw_alignment = {}
    fbrw_aligned = []
    undetermined_poses = {}
    success_poses = {}

    successes = []

    tof_vecs = []
    tof_readings_pts = []
    tof_readings_frames = []
    # _start_poses = []
    start_poses = {}

    for trial_name, trial_data in grouped_files.items():
        df_dict = {}
        for topic_name, data_file_path in trial_data.items():
            df_dict[topic_name] = pd.read_hdf(data_file_path[0])

        data_dict = {}
        for topic_name, data_df in df_dict.items():
            data_dict[topic_name] = data_df.to_dict(orient="list")

        try:
            start_pose = data_dict["trial_start_pose"]
            # start_poses[trial_name] = start_pose
        except KeyError:
            print(grouped_files)
            print("No start pose: ", trial_name)
            successes.append(0)
            continue

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
            successes.append(0)
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
            successes.append(0)
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
            successes.append(0)
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
            successes.append(1)
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

        (
            branch_center_point,
            branch_vec_normalized,
            tofA_vec,
            tofB_vec,
            tofA_reading_vec_base_frame,
            tofB_reading_vec_base_frame,
            A_sensor_id,
            B_sensor_id,
        ) = bp.get_branch_vec_from_tof(
            node=node, data_dict=df_dict, time_and_center_res_dict=time_and_center_res_dict, return_frames=True
        )

        tof_vecs.extend([tofA_vec, tofB_vec])
        tof_readings_pts.extend([tofA_reading_vec_base_frame, tofB_reading_vec_base_frame])
        tof_readings_frames.extend([A_sensor_id, B_sensor_id])
        start_poses[trial_name] = start_pose

        try:
            # fbrw_localization[trial_name] = df_dict["fbrw_controller_localization_success"].at[0, "controller_success"]
            # fbrw_alignment[trial_name] = df_dict["fbrw_controller_alignment_success"].at[0, "controller_success"]
            fbrw_localized.append(df_dict["fbrw_controller_localization_success"].at[0, "controller_success"])
            fbrw_aligned.append(df_dict["fbrw_controller_alignment_success"].at[0, "controller_success"])
            success_poses[trial_name] = start_pose
        except KeyError:
            print("No localization/alignment: ", trial_name)
            undetermined_poses[trial_name] = start_pose

        successes.append(2)

    return (
        tof_vecs,
        tof_readings_pts,
        tof_readings_frames,
        start_poses,
        successes,
        success_poses,
        undetermined_poses,
        fbrw_localized,
        fbrw_aligned,
    )


def main():
    __here__ = os.path.dirname(__file__)

    files: defaultdict = get_files()

    (
        tof_vecs,
        tof_readings_pts,
        tof_readings_frames,
        start_poses,
        successes,
        success_poses,
        undetermined_poses,
        localized,
        aligned,
    ) = get_projected_tof_readings(grouped_files=files)

    rows = []
    for trial_name, pose_dict in start_poses.items():
        row = {"trial": trial_name}
        row.update(pose_dict)
        rows.append(row)
    col_order = ["trial", "x", "y", "z", "qx", "qy", "qz", "qw"]
    start_poses_df = pd.DataFrame(columns=col_order, data=rows)
    start_poses_df.to_hdf(f"{__here__}/data/start_poses.h5", key="start_poses", format="fixed")

    success_scores = np.where(successes)[0]
    # failure_scores = ~success_scores

    tof_vecs = np.asarray(tof_vecs)
    tof_readings_pts = np.asarray(tof_readings_pts)

    # Fit
    centroid, direction = cf.fit_3d_linear_pca(points=tof_readings_pts)
    t_vals, coefs = cf.fit_3d_quadratic(points=tof_readings_pts, centroid=centroid, direction=direction)
    u_min, u_max = t_vals.min() - 1e-6, t_vals.max() + 1e-6

    quadratic_t_vals, quadratic_projected_points = cf.project_points_onto_curve(
        points=tof_readings_pts, t_vals=t_vals, coefs=coefs
    )

    with hpy.File(f"{__here__}/data/coefs.h5", "w") as f:
        f.create_dataset("coefs", data=coefs)

    df_branch_info = pd.DataFrame(columns=["cx", "cy", "cz", "dx", "dy", "dz"])
    df_branch_info.loc[0] = [centroid[0], centroid[1], centroid[2], direction[0], direction[1], direction[2]]
    df_branch_info.to_hdf(f"{__here__}/data/branch_info.h5", key="branch_info", format="fixed")

    df_successes = pd.DataFrame(columns=["success"], data=successes)
    df_successes.to_hdf(f"{__here__}/data/successes.h5", key="successes", format="fixed")

    df_localized = pd.DataFrame(columns=["localized"], data=localized)
    df_localized.to_hdf(f"{__here__}/data/localized.h5", key="localized", format="fixed")

    df_aligned = pd.DataFrame(columns=["aligned"], data=aligned)
    df_aligned.to_hdf(f"{__here__}/data/aligned.h5", key="aligned", format="fixed")

    rows = []
    for trial_name, pose_dict in success_poses.items():
        row = {"trial": trial_name}
        row.update(pose_dict)
        rows.append(row)

    # Ensure consistent column order
    col_order = ["trial", "x", "y", "z", "qx", "qy", "qz", "qw"]
    success_poses_df = pd.DataFrame(columns=col_order, data=rows)
    success_poses_df.to_hdf(f"{__here__}/data/success_poses.h5", key="success_poses", format="fixed")

    # with hpy.File(f"{__here__}/data/success_poses.h5", 'w') as f:
    #     f.create_dataset("success_poses", data=success_poses)
    rows = []
    for trial_name, pose_dict in undetermined_poses.items():
        row = {"trial": trial_name}
        row.update(pose_dict)
        rows.append(row)
    undetermined_poses_df = pd.DataFrame(columns=col_order, data=rows)
    undetermined_poses_df.to_hdf(f"{__here__}/data/undetermined_poses.h5", key="undetermined_poses", format="fixed")
    # with hpy.File(f"{__here__}/data/undetermined_poses.h5", 'w') as f:
    #     f.create_dataset("undetermined_poses", data=undetermined_poses)

    # Tests:
    residuals = tof_readings_pts[:, 0:3] - quadratic_projected_points
    # print("quadratic residuals:\n", residuals)
    print("quadratic RESIDUALS mean ", np.mean(np.linalg.norm(residuals, axis=1)))
    print("quadratic var:", np.var(residuals))
    print("quadratic std: ", np.std(residuals))

    quadratic_curve_df = pd.DataFrame(
        columns=[
            "tof_x",
            "tof_y",
            "tof_z",
            "tof_reading_x",
            "tof_reading_y",
            "tof_reading_z",
            "tof_frame",
            "t_vals",
            "quadratic_t_vals",
            "quadratic_projection_x",
            "quadratic_projection_y",
            "quadratic_projection_z",
            "residual_x",
            "residual_y",
            "residual_z",
        ],
        data=np.column_stack(
            (
                tof_vecs[:, 0],
                tof_vecs[:, 1],
                tof_vecs[:, 2],
                tof_readings_pts[:, 0],
                tof_readings_pts[:, 1],
                tof_readings_pts[:, 2],
                tof_readings_frames,
                t_vals,
                quadratic_t_vals,
                quadratic_projected_points[:, 0],
                quadratic_projected_points[:, 1],
                quadratic_projected_points[:, 2],
                residuals[:, 0],
                residuals[:, 1],
                residuals[:, 2],
            )
        ),
    )
    quadratic_curve_df.to_hdf(f"{__here__}/data/quadratic_curve.h5", key="data", format="fixed")

    generated_pt_scores = np.zeros(shape=len(start_poses), dtype=float)

    positions = []
    localized = []
    aligned = []

    fig = go.Figure()
    plot_t_vals = np.linspace(sorted(t_vals)[0], sorted(t_vals)[-1], len(t_vals))
    tube_mesh_info = ph.get_tube_mesh_info(coefs=coefs, radius=0.0065, u_vals=plot_t_vals)
    fig = ph.plot_tube_mesh(fig=fig, tube_mesh_info=tube_mesh_info, name='branch')
    

    for i, (trial_name, pose) in enumerate(start_poses.items()):
        rot_mat = Rotation.from_quat([pose["qx"][0], pose["qy"][0], pose["qz"][0], pose["qw"][0]]).as_matrix()
        ori_vec = rot_mat @ [0, 0, 1]
        ori_vec /= np.linalg.norm(ori_vec)
        pos = [pose["x"][0], pose["y"][0], pose["z"][0]]
        positions.append(pos)

        sensor_fov_deg = 18
        sigma_deg = 18 / 3

        rotated_fov_pts = rt.generate_cylindrical_pts(
            r_range=(0.04891, 0.04891),
            theta_range=(0, 2 * np.pi),
            z_range=(0, 0),
            num_r_pts=1,
            num_theta_pts=30,
            num_z_pts=1,
            start_point=pos,
            start_orientation=ori_vec,
        )
        
        

        rotated_pt_scores = np.zeros(shape=len(rotated_fov_pts), dtype=float)

        for j, rotated_pt in enumerate(rotated_fov_pts):
            v_vec = np.cross(ori_vec, [0, 0, 1])
            w_vec = np.cross(ori_vec, v_vec)
            sampled_directions = rt.sample_gaussian_cone(
                u=ori_vec, v=v_vec, w=w_vec, sensor_fov_deg=sensor_fov_deg, sigma_deg=sigma_deg, num_samples=1000
            )

            scored_directions = rt.score_quadratic_directions(
                start_point=pos,
                directions=sampled_directions,
                coefs=coefs,
                branch_radius=0.0065,
                lambda_dist=0.25,
                u_min=u_min,
                u_max=u_max,
            )

            if not np.any(scored_directions):
                pass

            rotated_pt_scores = np.mean(scored_directions)

        generated_pt_scores[i] = np.mean(rotated_pt_scores)

        if not np.any(rotated_pt_scores):
            # print(sampled_directions)
            fig.add_trace(
                go.Scatter3d(
                    x=[pos[0]],
                    y=[pos[1]],
                    z=[pos[2]],
                )
            )
            start_pts_repeated = np.tile(pos, (len(sampled_directions), 1))
            new_ps = start_pts_repeated + 0.1 * sampled_directions
            print(start_pts_repeated)
            print(new_ps)
            fig.add_trace(
                go.Scatter3d(
                    x=[start_pts_repeated[:, 0], new_ps[:, 0]],
                    y=[start_pts_repeated[:, 1], new_ps[:, 1]],
                    z=[start_pts_repeated[:, 2], new_ps[:, 2]],
                    mode='lines'
                )
            )

    generated_pt_scores = np.sqrt(generated_pt_scores)

    positions = np.asarray(positions)
    fig.add_trace(
        go.Scatter3d(
            x=positions[:, 0],
            y=positions[:, 1],
            z=positions[:, 2],
            mode='markers',
            marker=dict(color='#000000')
        )
    )
    fig.update_layout(scene=dict(aspectmode='data'))
    fig.show()
    import sys
    sys.exit()

    with hpy.File(f"{__here__}/data/rt_scores.h5", "w") as f:
        f.create_dataset("rt_scores", data=generated_pt_scores)

    ############################
    # Plot positions
    ##############################
    # positions = np.asarray(positions)
    # fig = go.Figure()
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=positions[:, 0],
    #         y=positions[:, 1],
    #         z=positions[:, 2],
    #         name='scored_start_poses',
    #         mode='markers',
    #         marker=dict(size=4, color=generated_pt_scores, colorscale="matter_r", opacity=0.7, showscale=True),

    #     )
    # )

    ################################
    # Plot orientation vectors
    ################################
    #     # fig.add_trace(
    #     #     go.Scatter3d(
    #     #         x=pos[0],
    #     #         y=pos[1],
    #     #         z=pos[2],
    #     #         mode='markers',
    #     #         marker=dict(size=4),
    #     #         name=trial_name,
    #     #     )
    #     # )
    #     fig = ph.plot_vector(
    #         fig=fig,
    #         position=pos,
    #         orientation=ori_vec,
    #         color='blue',
    #         name='orientation',
    #         showlegend=True,
    #         scale=0.1
    #     )
    # fig.show()

    #####################################
    # Plot tof readings
    #######################################
    # fig = go.Figure()
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=tof_readings_pts[:,0],
    #         y=tof_readings_pts[:,1],
    #         z=tof_readings_pts[:,2],
    #         mode='markers',
    #         marker=dict(size=4)
    #     )
    # )

    ########################################
    # Plot quadratic fits and residuals
    #######################################3
    # fig = pb.plot_quadratic_fit(t_vals=quadratic_t_vals, coefs=coefs, fig=fig)
    # # fig = pb.plot_quadratic_residuals(points=tof_readings_pts, projected_points=quadratic_projected_points, fig=fig)
    # fig.update_layout(scene=dict(aspectmode='data'))
    # fig.show()

    #######################################
    # Plot binned difficulty scores
    #######################################

    print(len(success_scores))
    # print(len(failure_scores))
    print(len(generated_pt_scores))

    # generated_pt_scores_successes = generated_pt_scores[success_scores]
    # generated_pt_scores_failures = generated_pt_scores_failures[failure_scores]

    # _success = successes[success_scores]
    # _failure = successes[failure_scores]

    fig = go.Figure()
    # fig.add_trace(
    #     go.Scatter(
    #         x=generated_pt_scores_successes,
    #         y=_success,
    #         mode='markers',
    #         color="#3AB734",
    #         marker=dict(size=4)
    #     )
    # )
    # fig.add_trace(
    #     go.Scatter(
    #         x=generated_pt_scores_failures,
    #         y=_failure,
    #         mode='markers',
    #         color="#E22B25",
    #         marker=dict(size=4)
    #     )
    # )
    fig.add_trace(
        go.Scatter(
            x=generated_pt_scores,
            y=successes,
            mode="markers",
            marker=dict(size=4, color="#3ab734"),
        )
    )
    fig.show()

    return


if __name__ == "__main__":
    main()
