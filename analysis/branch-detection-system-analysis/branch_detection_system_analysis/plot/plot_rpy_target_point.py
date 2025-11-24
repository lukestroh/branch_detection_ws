#!/usr/bin/env python3
from branch_detection_system_analysis.fit import branch_processing as bp
from branch_detection_system_analysis.node import dummy_node
from branch_detection_system_analysis.plot import plotting_backend as pb
from branch_detection_system_analysis.plot import debug_plots as dplot
from branch_detection_system_analysis.plot import plotly_helpers as ph
from final_approach_controller import curve_fitting as cf
from final_approach_controller import data_processing as dp
import numpy as np
import os
import pandas as pd
import plotly.graph_objects as go
import pprint as pp

from scipy.spatial.transform import Rotation


def main():
    node = dummy_node.DummyNode()

    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))

    data_dict = {}
    files_by_date = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250805")

    files_generated_start_poses = pb.filter_files_by_topic(files=files_by_date, topic="generated_start_poses")

    for file in files_generated_start_poses:
        df_generated_start_poses = pd.read_hdf(file)
        if not df_generated_start_poses.empty:
            break

    files_all_topics = pb.filter_files_by_topics(
        files=files_by_date,
        topics=[
            "tf",
            "tf_static",
            "trial_start_pose",
            "tof0_filtered",
            "tof1_filtered",
            "fbrw_controller_alignment_success",
            "fbrw_controller_localization_success",
            "rotation_started",
            "rotation_stopped",
            "trial_start_pose",
            "trial_start_pose_index",
            "joint_states",
            "rpy_target_pose",
        ],
    )

    grouped_files = pb.group_files_by_datetime_by_topic(files=files_all_topics)

    center_pts = []
    pose_idxs = []
    tof_readings_pts = []
    tof_locations = []
    fig = go.Figure()

    global_minima_counts = np.zeros(4)
    sus_trial_names = []

    for j, (trial_name, trial_data) in enumerate(grouped_files.items()):
        trial_pose_idx = pd.read_hdf(trial_data['trial_start_pose_index'][0])
        # trial_pose_idx = trial_pose_idx.at[0, 'pose_index']
        # print(trial_pose_idx)
        df_dict = {}
        for topic_name, data_file_path in trial_data.items():
            df_dict[topic_name] = pd.read_hdf(data_file_path[0])

        data_dict = {}
        for topic_name, data_df in df_dict.items():
            data_dict[topic_name] = data_df.to_dict(orient="list")

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

        separated_data_dict, minima_counts = dp.separate_tof_data_by_curve(
            node=node, all_data_dict=all_data_dict, save_fig=False, show_fig=False
        )
        if minima_counts[1]:
            sus_trial_names.append(trial_name)
        global_minima_counts += minima_counts

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

        center_pts.append(branch_center_point)
        pose_idxs.append(trial_pose_idx.at[0, 'pose_index'])

        rpy_target_pose = data_dict["rpy_target_pose"]
        # print()
        # pose_idxs.append(data_dict['trial_start_pose_index']['pose_index'][0][0])
        # # if data_dict['trial_start_pose_index']['pose_index'][0][0] == 42:
        # pose = data_dict['trial_start_pose']
        # pos = [pose['x'][0], pose['y'][0], pose['z'][0]]
        # ori = [pose['qx'][0], pose['qy'][0], pose['qz'][0], pose['qw'][0]]

        # fig = ph.plot_vector(
        #     fig=fig,
        #     position=pos,
        #     orientation=ori,
        #     scale=0.1,
        #     color='blue',
        #     name=data_dict['trial_start_pose_index']['pose_index'][0][0],
        #     showlegend=True
        # )
    
    print(f"GLOBAL MINIMA COUNTS: {global_minima_counts}")
    pp.pprint(sus_trial_names)
    print(len(sus_trial_names))
    import sys
    sys.exit()

    center_pts = np.asarray(center_pts)[:, :3]
    print(center_pts)
    center_mean = np.mean(center_pts, axis=0)

    pose_idxs = np.asarray(pose_idxs).flatten() - 1
    print(pose_idxs)

    fig = go.Figure()
    start_poses = []
    start_oris = []
    for i, row in df_generated_start_poses.iterrows():

        pos = [float(row["x"]), float(row["y"]), float(row["z"])]
        start_poses.append(pos)
        
        fig.add_trace(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], name=i))

        q = [row["qx"], row["qy"], row["qz"], row["qw"]]
        rot_mat = Rotation.from_quat(q).as_matrix()

        ori = rot_mat @ [0, 0, 1]
        ori /= np.linalg.norm(ori)

        start_oris.append(ori)

        if i == 70:

            q = [row["qx"], row["qy"], row["qz"], row["qw"]]
            rot_mat = Rotation.from_quat(q).as_matrix()

            ori = rot_mat @ [0, 0, 1]
            ori /= np.linalg.norm(ori)

            target_pos = pos + ori * 0.1


    start_poses = np.asarray(start_poses)
    cross_vec = start_poses[0] - start_poses[-1]
    cross_vec = cross_vec / np.linalg.norm(cross_vec)
    # import sys
    # sys.exit()
    start_oris = np.asarray(start_oris)
    used_start_oris = start_oris[pose_idxs]
    perp_vec = np.cross(cross_vec, start_oris[0])

    print("START")

    view_dir_rms_errors = []
    perp_dir_rms_errors = []
    u_dir_rms_errors = []
    view_dir_mean_errors = []
    perp_dir_mean_errors = []
    u_dir_mean_errors = []

    for i in range(5):
        grouped_idxs = np.where((pose_idxs >= i * 25) & (pose_idxs < (i+1) * 25))
        print(used_start_oris[grouped_idxs])
        print(grouped_idxs)
        residual_vecs = center_pts[grouped_idxs] - target_pos
        print(residual_vecs)

        u_dir = np.cross(perp_vec, used_start_oris[grouped_idxs][0])
        u_dir = u_dir / np.linalg.norm(u_dir)

        # 1. RMS viewdir error (range accuracy)
        viewdir_error = np.sum(np.multiply(residual_vecs, used_start_oris[grouped_idxs]), axis=1)[:, np.newaxis]
        rms_viewdir = np.sqrt(np.mean(viewdir_error**2))
        print("RMS viewdir ERROR")
        print(rms_viewdir)
        mean_viewdir_error = np.mean(viewdir_error) 
        # 2. perpendicular error (up/down from branch)
        perp_error = np.sum(np.multiply(residual_vecs, perp_vec), axis=1)[:, np.newaxis]
        # print("PERP ERROR:")
        # print(perp_error)
        rms_perp = np.sqrt(np.mean(perp_error**2))
        print("RMS PERP_ERROR")
        print(rms_perp)
        perp_mean_error = np.mean(perp_error)

        # 2. perpendicular error (up/down from branch)
        u_error = np.sum(np.multiply(residual_vecs, u_dir), axis=1)[:, np.newaxis]
        # print("PERP ERROR:")
        # print(perp_error)
        rms_u = np.sqrt(np.mean(u_error**2))
        print("RMS u_ERROR")
        print(rms_u)
        u_mean_error = np.mean(u_error)

        view_dir_rms_errors.append(rms_viewdir)
        perp_dir_rms_errors.append(rms_perp)
        u_dir_rms_errors.append(rms_u)
        view_dir_mean_errors.append(mean_viewdir_error)
        perp_dir_mean_errors.append(perp_mean_error)
        u_dir_mean_errors.append(u_mean_error)

    print(view_dir_rms_errors)
    print(perp_dir_rms_errors)
    print(u_dir_rms_errors)
    print(view_dir_mean_errors)
    print(perp_dir_mean_errors)
    print(u_dir_mean_errors)

    print("ALL MEANS")
    print(np.mean(view_dir_rms_errors))
    print(np.mean(perp_dir_rms_errors))
    print(np.mean(u_dir_rms_errors))
    print(np.mean(view_dir_mean_errors))
    print(np.mean(perp_dir_mean_errors))
    print(np.mean(u_dir_mean_errors))
    # used_start_poses = start_poses[pose_idxs]
    # print(used_start_poses)

    # print("TARGET_POS")
    # print(target_pos)

    view_vecs = target_pos - center_pts
    # print(view_vecs)
    # import sys
    # sys.exit()


    
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=[target_pos[0]],
    #         y=[target_pos[1]],
    #         z=[target_pos[2]],
    #         name='target_pt',
    #         mode='markers',
    #         marker=dict(color='orange')
    #     )
    # )
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=center_pts[:,0],
    #         y=center_pts[:,1],
    #         z=center_pts[:,2],
    #         mode='markers',
            
    #     )
    # )
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=[start_poses[0,0], start_poses[-1,0]],
    #         y=[start_poses[0,1], start_poses[-1,1]],
    #         z=[start_poses[0,2], start_poses[-1,2]],
    #     )
    # )
    # fig.update_layout(scene=dict(aspectmode='data'))
    # fig.show()

    # for i, row in df_generated_start_poses.iterrows():
    #     if i == 20:
    #         pos1 = np.asarray([float(row["x"]), float(row["y"]), float(row["z"])])
    #     if i == 70:
    #         pos2 = np.asarray([float(row["x"]), float(row["y"]), float(row["z"])])

    # v = target_pos - pos2
    # v /= np.linalg.norm(v)
    # aa = pos1 - pos2
    # aa /= np.linalg.norm(aa)
    # w = np.cross(aa, v)
    # w /= np.linalg.norm(w)
    # u = np.cross(v, w)
    # u /= np.linalg.norm(u)

    # basis_trial = np.identity(4)
    # basis_trial[:3, 3] = center_mean
    # basis_trial[:3, :3] = np.column_stack((u, v, w))

    # basis_inv = np.linalg.inv(basis_trial)

    # pts = (basis_inv @ np.column_stack((center_pts, np.ones(len(center_pts)))).T).T[:, :3]
    # print(pts)

    # target__pt = (basis_inv @ np.concatenate((target_pos, [1])).T).T[:3]
    # print(target__pt)

    # center_mean = np.mean(pts, axis=0)
    # center_diffs = pts - center_mean
    # means = np.abs(center_diffs).mean(axis=0)  # mean per component
    # stds = np.abs(center_diffs).std(axis=0)  # std per component
    # print("---------------------------------")
    # print("mean xyz: ", means)
    # print("std xyz: ", stds)

    # lengths = np.linalg.norm(center_diffs, axis=1)
    # mean_len = lengths.mean()
    # std_len = lengths.std()
    # print("Mean center length:", mean_len)
    # print("Std dev of lengths  :", std_len)

    # diff = target__pt - center_mean
    # print("target - calculated: ", np.abs(target__pt - center_mean))
    # print("mag: ", np.linalg.norm(diff))

    # fig.add_trace(go.Scatter3d(x=pts[:, 0], y=pts[:, 1], z=pts[:, 2], mode="markers", name="pts"))
    # fig.add_trace(go.Scatter3d(x=[means[0]], y=[means[1]], z=[means[2]], mode="markers", name="mean"))

    # # fig = ph.plot_vector(fig=fig, position=target_pos, orientation=u, color="#D01C1C", scale=0.1)
    # # fig = ph.plot_vector(fig=fig, position=target_pos, orientation=v, color="#2E9E1F", scale=0.1)
    # # fig = ph.plot_vector(fig=fig, position=target_pos, orientation=w, color="#263ABD", scale=0.1)

    # fig.update_layout(scene=dict(aspectmode="data"))
    # fig.show()


if __name__ == "__main__":
    main()
