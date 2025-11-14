#!/usr/bin/env python3
from branch_detection_system_analysis.fit import branch_processing as bp
from branch_detection_system_analysis.node import dummy_node
from branch_detection_system_analysis.plot import plotting_backend as pb
from branch_detection_system_analysis.plot import plotly_helpers as ph
from final_approach_controller import curve_fitting as cf
from final_approach_controller import data_processing as dp
import numpy as np
import os
import pandas as pd
import plotly.graph_objects as go
import pprint as pp

import h5py as hpy
__here__ = os.path.dirname(__file__)


def main():
    node = dummy_node.DummyNode()

    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))

    data_dict = {}
    files_by_date = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250729") + pb.get_files_by_date(
        warehouse_path=warehouse_path, date="20250730"
    )

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
            "trial_start_pose_index",
            "tof0_filtered",
            "tof1_filtered",
            "fbrw_controller_alignment_success",
            "fbrw_controller_localization_success",
            "rotation_started",
            "rotation_stopped",
            "joint_states",
        ],
    )

    grouped_files = pb.group_files_by_datetime_by_topic(files=files_all_topics)

    print(len(grouped_files))
    import sys
    sys.exit()

    tof_readings_pts = []
    tof_locations = []

    for trial_name, trial_data in grouped_files.items():
        df_dict = {}
        for topic_name, data_file_path in trial_data.items():
            df_dict[topic_name] = pd.read_hdf(data_file_path[0])

        data_dict = {}
        for topic_name, data_df in df_dict.items():
            data_dict[topic_name] = data_df.to_dict(orient="list")

        print(data_dict["trial_start_pose"])
        print(data_dict["trial_start_pose_index"])
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
        tof_locations.extend([tofA_vec, tofB_vec])
        tof_readings_pts.extend([tofA_reading_vec_base_frame, tofB_reading_vec_base_frame])

    tof_readings_pts = np.asarray(tof_readings_pts)
    tof_locations = np.asarray(tof_locations)

    print(len(tof_readings_pts))
    

    assert tof_readings_pts.shape == tof_locations.shape

    # Fit
    centroid, direction = cf.fit_3d_linear_pca(points=tof_readings_pts)
    t_vals, coefs = cf.fit_3d_quadratic(points=tof_readings_pts, centroid=centroid, direction=direction)

    # file_coefs = hpy.Flie(f'{__here__}/data/detection_coefs.h5', 'w')


    quadratic_t_vals, quadratic_projected_points = cf.project_points_onto_curve(
        points=tof_readings_pts, t_vals=t_vals, coefs=coefs
    )

    def get_quadratic_deriv_coefs(quad_coefs: np.ndarray):
        d_coefs = np.column_stack((2 * quad_coefs[:, 0], quad_coefs[:, 1]))
        return d_coefs

    def get_quadratic_deri_vals(quad_coefs: np.ndarray, t_vals: np.ndarray):
        d_coefs = get_quadratic_deriv_coefs(quad_coefs=quad_coefs)
        d_xyz = np.outer(d_coefs[:, 0], t_vals).T + d_coefs[:, 1]
        return d_xyz

    # Tests:
    residual_vecs = tof_readings_pts[:, 0:3] - quadratic_projected_points
    d_xyz = get_quadratic_deri_vals(quad_coefs=coefs, t_vals=quadratic_t_vals)
    u = (-1 * d_xyz) / np.linalg.norm(d_xyz)

    w = tof_readings_pts[:, :3] - tof_locations[:, :3]
    w /= np.linalg.norm(w, axis=1, keepdims=True)
    v = np.cross(w, u)
    v /= np.linalg.norm(v, axis=1, keepdims=True)

    u_viewdir = np.cross(v, w)
    
    print("RESIDUAL VECS:")
    print(residual_vecs)
    print("W:")
    print(w)

    # 1. RMS viewdir error (range accuracy)
    viewdir_error = np.sum(np.multiply(residual_vecs, w), axis=1)[:, np.newaxis]
    # print("viewdir ERROR:")
    # print(viewdir_error)
    rms_viewdir = np.sqrt(np.mean(viewdir_error**2))
    print("RMS viewdir ERROR")
    print(rms_viewdir)
    # Look for systematic bias
    mean_viewdir_error = np.mean(viewdir_error)  # should be near zero if unbiased
    print("MEAN viewdir ERROR")
    print(mean_viewdir_error)
    print("STD viewdir ERROR:")
    print(np.std(viewdir_error))

    # 2. perpendicular error (up/down from branch)
    perp_error = np.sum(np.multiply(residual_vecs, v), axis=1)[:, np.newaxis]
    # print("PERP ERROR:")
    # print(perp_error)
    rms_perp = np.sqrt(np.mean(perp_error**2))
    print("RMS PERP_ERROR")
    print(rms_perp)
    perp_mean_error = np.mean(perp_error)
    print("MEAN PERP ERROR:")
    print(perp_mean_error)

    u_viewdir_error = np.sum(np.multiply(residual_vecs, u_viewdir), axis=1)[:,np.newaxis]
    rms_u_viewdir = np.sqrt(np.mean(u_viewdir_error**2))

    # Angle between view direction and curve tangent
    angle = np.arccos(np.clip(np.einsum("ij,ij->i", w, d_xyz), -1, 1))
    # print("ANGLE:")
    # print(np.degrees(angle))
    print("ANGLE MEAN:")
    print(np.mean(np.degrees(angle)))

    # 3. Total RMS error
    rms_total = np.sqrt(np.mean(np.linalg.norm(residual_vecs, axis=1) ** 2))
    print("RMS TOTAL:")
    print(rms_total)
    print(np.sqrt(rms_perp**2 + rms_viewdir**2 + rms_u_viewdir**2))

    # 4. Max error (outlier detection)
    max_error = np.max(np.linalg.norm(residual_vecs, axis=1))
    print("MAX_ERROR:")
    print(max_error)


    fig = go.Figure()
    fig.add_trace(go.Scatter(x=np.degrees(angle), y=viewdir_error.flatten(), mode="markers", marker=dict(size=10)))
    fig.show()

    print("EHLLO WORLD")

    fig = go.Figure()
    for i in range(tof_readings_pts.shape[0]):
        fig.add_trace(
            go.Scatter3d(
                mode='markers+lines',
                x=[tof_readings_pts[i,0], tof_locations[i,0]],
                y=[tof_readings_pts[i,1], tof_locations[i,1]],
                z=[tof_readings_pts[i,2], tof_locations[i,2]],
            )
        )
        fig.add_trace(
            go.Scatter3d(
                mode='markers+lines',
                x=[tof_locations[i,0]],
                y=[tof_locations[i,1]],
                z=[tof_locations[i,2]],
                marker=dict(color='red')
            )
        )
    fig = pb.plot_quadratic_fit(t_vals=quadratic_t_vals, coefs=coefs, fig=fig)
    fig.add_trace(
        go.Scatter3d(
            x=tof_locations[:, 0],
            y=tof_locations[:, 1],
            z=tof_locations[:, 2],
            mode='markers'
        )
    )
    for i, pos in enumerate(quadratic_projected_points):
        # fig = ph.plot_vector(fig=fig, position=pos, orientation=basis_matrices[i][0], scale=0.1, color="#D01919")
        fig = ph.plot_vector(fig=fig, position=pos, orientation=v[i], scale=0.05, color="#19D022")
        fig = ph.plot_vector(fig=fig, position=pos, orientation=w[i], scale=0.05, color="#1922D0")
    fig.update_layout(scene=dict(aspectmode='data'))
    fig.show()

    return


if __name__ == "__main__":
    main()
