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


def generate_uniform_cylindrical_position_poses(generate_poses_goal: GenerateCylindricalPoses.Goal) -> list[Pose]:
    poses = []

    r = np.linspace(
        start=generate_poses_goal.radius_range[0],
        stop=generate_poses_goal.radius_range[1],
        num=generate_poses_goal.num_radius_poses,
    )
    theta = np.linspace(
        start=generate_poses_goal.theta_range[0],
        stop=generate_poses_goal.theta_range[1],
        num=generate_poses_goal.num_theta_poses,
    )
    x = np.outer(r, np.cos(theta)).flatten()
    y = np.outer(r, np.sin(theta)).flatten()

    z = np.linspace(
        start=generate_poses_goal.z_range[0],
        stop=generate_poses_goal.z_range[1],
        num=generate_poses_goal.num_z_poses,
    )

    xy = np.stack((x, y), axis=1)
    xy_repeated = np.tile(xy, reps=(len(z), 1))
    z_repeated = np.repeat(z, len(x))[:, np.newaxis]

    xyz = np.hstack((xy_repeated, z_repeated))

    for p in xyz:
        poses.append(Pose(position=Point(x=p[0], y=p[1], z=p[2])))

    return poses


def pose_to_tf_mat(pose: Pose):
    mat = np.identity(4)
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    mat[:3, :3] = Rotation.from_quat(
        quat=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    ).as_matrix()
    return mat


def poses_to_points(poses: list[Pose]):
    points = np.ones(shape=(len(poses), 4), dtype=float)
    for i in range(len(poses)):
        points[i, 0] = poses[i].position.x
        points[i, 1] = poses[i].position.y
        points[i, 2] = poses[i].position.z
    return points


def get_poses():
    start_pose = Pose(
        position=Point(x=-0.336062743489267, y=1.070126103681655, z=1.7842673493199608),
        orientation=Quaternion(
            x=0.7446764785111847, y=-0.03594632354223663, z=0.09850742246533545, w=-0.6591366261217881
        ),
    )

    tf_mat = pose_to_tf_mat(pose=start_pose)

    goal = GenerateCylindricalPoses.Goal()
    # Edge case settings
    goal.num_radius_poses = 5
    goal.num_theta_poses = 18
    goal.num_z_poses = 5
    goal.radius_range = [0.02, 0.09]
    goal.theta_range = [0.0, 2 * np.pi]
    goal.z_range = [0.0, -0.20]

    uniform_poses = generate_uniform_cylindrical_position_poses(generate_poses_goal=goal)
    points = poses_to_points(poses=uniform_poses)
    transformed_points = points @ tf_mat.T
    ids = np.arange(len(transformed_points))

    # transformed_points = transformed_points[:61]

    # fig = go.Figure()
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=transformed_points[:, 0],
    #         y=transformed_points[:, 1],
    #         z=transformed_points[:, 2],
    #         name="generated_pts",
    #         mode="markers",
    #         marker=dict(size=4),
    #         customdata=ids.reshape(-1,1),
    #         hovertemplate='ID: %{customdata[0]}<br>x:%{x:.3f}<br>y:%{y:.3f}<br>z:%{z:.3f}<extra></extra>'
    #     )
    # )
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=[tf_mat[0, 3]],
    #         y=[tf_mat[1, 3]],
    #         z=[tf_mat[2, 3]],
    #         name="generation_start_point",
    #     ),

    # )
    # fig.update_layout(scene=dict(aspectmode='data'))
    # fig.show()

    return transformed_points


def main():
    ws_path = os.path.abspath(os.path.join("/home/luke/branch_detection_ws"))
    warehouse_path = os.path.abspath(os.path.join(ws_path, "bags", "2025_ToFBranchDetection", "warehouse"))

    # data_dict = {}
    # files_by_date = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250729")

    # files_tf = pb.filter_files_by_topic(files=files_by_date, topic='tf')
    # files_tf_static = pb.filter_files_by_topic(files=files_by_date, topic='tf_static')
    # files_generated_start_poses = pb.filter_files_by_topic(files=files_by_date, topic="generated_start_poses")
    # files_start_position = pb.filter_files_by_topic(files=files_by_date, topic='trial_start_pose')
    # files_start_position_idx = pb.filter_files_by_topic(files=files_by_date, topic='trial_start_pose_index')
    # files_joint_states = pb.filter_files_by_topic(files=files_by_date, topic='joint_states')
    # files_rotation_started = pb.filter_files_by_topic(files=files_by_date, topic='rotation_started')

    # start_points = get_poses()

    # last_d = None
    # j = 0

    # fig = go.Figure()

    # _idx = 26

    # files_tf = files_tf[_idx:]
    # files_tf_static = files_tf_static[_idx:]
    # files_start_position = files_start_position[_idx:]
    # files_start_position_idx = files_start_position_idx[_idx:]
    # files_joint_states = files_joint_states[_idx:]

    # tf_vecs = []
    # js = []
    # dts = []

    # files_tf = files_tf[26:]

    start_trial_date = "20250729_22-25-35"

    ###########################################################################################################
    # FIXES 07-29 trials
    #########################################################################################################
    # for i in range(len(files_tf)):
    #     file_date = files_tf[i].split('__')[-3].replace('_0', '')
    #     d = dt.datetime.strptime(file_date, "%Y%m%d_%H-%M-%S")

    #     rotation_started_ts = pd.read_hdf(files_rotation_started[i]).at[0, 'rotation_event_ts']
    #     tf_df = pd.read_hdf(files_tf[i])
    #     tf_static_df = pd.read_hdf(files_tf_static[i])
    #     joint_states_df = pd.read_hdf(files_joint_states[i])

    #     # start_position_df = pd.read_hdf(files_start_position[i])
    #     # start_position_idx_df = pd.read_hdf(files_start_position_idx[i])

    # tf_df = pb.get_tf_df_at_closest_timestamp(tf_df=tf_df, tf_static_df=tf_static_df, timestamp=rotation_started_ts)
    # tf_tof_to_base = pb.get_tf_matrix_from_df(
    #     target_frame=f"amiga__base",
    #     source_frame=f"mock_pruner__tool0",
    #     tf_df=tf_df,
    # )
    # vec = tf_tof_to_base @ [0, 0, 0, 1]
    # ori = Rotation.from_matrix(matrix=tf_tof_to_base[:3, :3])
    # q = ori.as_quat()

    #     start_position_df = pd.DataFrame(columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    #     start_position_df.loc[-1] = [vec[0], vec[1], vec[2], q[0], q[1], q[2], q[3]]
    #     start_position_df.index = start_position_df.index + 1
    #     start_position_df.to_hdf(files_start_position[i], key='data', format='fixed')

    #     start_position_idx_df = pd.DataFrame(columns=['pose_index'])
    #     start_position_idx_df.loc[-1] = [i]
    #     start_position_idx_df.index = start_position_idx_df.index + 1
    #     start_position_idx_df.to_hdf(files_start_position_idx[i], key='data', format='fixed')
    ################################################################################################
    # tf_vecs.append(vec)

    # joint_states_df = pb.get_df_rows_at_closest_timestamp(df=joint_states_df, topic_name="joint_states", timestamps=[rotation_started_ts])

    # joint_states = joint_states_df.at[0, "joint_states_pos"]
    # if i == 0:
    #     start_js = joint_states

    # if np.all(np.isclose(start_js, joint_states, atol=0.005)):
    #     j=0
    # else:
    #     j += 1
    # # print(joint_states, j)
    #     js.append(j)
    #     dts.append(d.strftime("%Y%m%d_%H-%M-%S"))

    # tf_vecs = np.asarray(tf_vecs)
    # tf_vecs = tf_vecs[_idx:]
    # js = np.asarray(js)[_idx:]
    # js = js - js[0]
    # dts = np.asarray(dts)[_idx:]

    # gen_pts = start_points[js]
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=gen_pts[:, 0],
    #         y=gen_pts[:, 1],
    #         z=gen_pts[:, 2],
    #         mode='markers',
    #         marker=dict(size=4),
    #         name="gen_pts",
    #         customdata=np.column_stack((js.reshape(-1,1),)),
    #         hovertemplate='ID: %{customdata[0]}<br>x:%{x:.3f}<br>y:%{y:.3f}<br>z:%{z:.3f}<extra></extra>'
    #     )
    # )
    # fig.add_trace(
    #     go.Scatter3d(
    #         x=tf_vecs[:, 0],
    #         y=tf_vecs[:, 1],
    #         z=tf_vecs[:, 2],
    #         mode='markers',
    #         marker=dict(size=4),
    #         name="tf_vecs",
    #         customdata=np.column_stack((js.reshape(-1,1), np.asarray(dts).reshape(-1,1))),
    #         hovertemplate='ID: %{customdata[0]}<br>date: %{customdata[1]}<br>x:%{x:.3f}<br>y:%{y:.3f}<br>z:%{z:.3f}<extra></extra>'
    #     )
    # )
    # fig.update_layout(scene=dict(aspectmode='data'))
    # fig.show()

    files_by_date = pb.get_files_by_date(warehouse_path=warehouse_path, date="20250730")

    files_tf = pb.filter_files_by_topic(files=files_by_date, topic="tf")
    files_tf_static = pb.filter_files_by_topic(files=files_by_date, topic="tf_static")
    files_generated_start_poses = pb.filter_files_by_topic(files=files_by_date, topic="generated_start_poses")
    files_start_position = pb.filter_files_by_topic(files=files_by_date, topic="trial_start_pose")
    files_start_position_idx = pb.filter_files_by_topic(files=files_by_date, topic="trial_start_pose_index")
    files_joint_states = pb.filter_files_by_topic(files=files_by_date, topic="joint_states")
    files_rotation_started = pb.filter_files_by_topic(files=files_by_date, topic="rotation_started")
    files_fbrw_controller_alignment_success = pb.filter_files_by_topic(
        files_by_date, topic="fbrw_controller_alignment_success"
    )
    files_fbrw_controller_localization_success = pb.filter_files_by_topic(
        files_by_date, topic="fbrw_controller_localization_success"
    )
    files_tof0 = pb.filter_files_by_topic(files=files_by_date, topic="tof0_filtered")
    files_tof1 = pb.filter_files_by_topic(files=files_by_date, topic="tof1_filtered")

    gen_pts = get_poses()
    gen_pts = gen_pts[:, :3]

    start_points = []
    tf_points = []
    start_points_idxs = []
    dts = []
    seen_idxs = []
    start_pose_idx = 41
    # pp.pprint(files_start_position)
    for i in range(len(files_start_position)):
        file_date = files_tf[i].split("__")[-3].replace("_0", "")
        d = dt.datetime.strptime(file_date, "%Y%m%d_%H-%M-%S")
        dts.append(d)

        tf_df = pd.read_hdf(files_tf[i])
        tf_static_df = pd.read_hdf(files_tf_static[i])
        start_pos_df = pd.read_hdf(files_start_position[i])
        start_pos_idx_df = pd.read_hdf(files_start_position_idx[i])
        # print(start_pos_df)
        # print(start_pos_idx_df)
        if start_pos_df.empty:
            ...
            continue
            # print("start pos empty!")
            # rotation_started_ts = pd.read_hdf(files_rotation_started[i]).at[0, 'rotation_event_ts']
            # tf_df = pb.get_tf_df_at_closest_timestamp(tf_df=tf_df, tf_static_df=tf_static_df, timestamp=rotation_started_ts)
            # tf_tof_to_base = pb.get_tf_matrix_from_df(
            #     target_frame=f"amiga__base",
            #     source_frame=f"mock_pruner__tool0",
            #     tf_df=tf_df,
            # )
            # vec = tf_tof_to_base @ [0, 0, 0, 1]
            # ori = Rotation.from_matrix(matrix=tf_tof_to_base[:3, :3])
            # q = ori.as_quat()

            # start_points.append(vec[:3])

            # _start_position_df = pd.DataFrame(columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            # _start_position_df.loc[-1] = [vec[0], vec[1], vec[2], q[0], q[1], q[2], q[3]]
            # _start_position_df.index = _start_position_df.index + 1
            # _start_position_df.to_hdf(files_start_position[i], key='data', format='fixed')

        else:
            start_pt = [
                start_pos_df.at[0, "x"],
                start_pos_df.at[0, "y"],
                start_pos_df.at[0, "z"],
            ]
            start_points.append(start_pt)

        matches = np.all(np.isclose(gen_pts, start_pt, atol=0.005), axis=1)
        index = np.where(matches)[0]
        print(index)

        if index.shape[0] == 2:
            if index[0] in seen_idxs:
                _index = index[1]
            else:
                _index = index[0]

        else:
            _index = index[0]
        seen_idxs.append(_index)
        # print(_index)

        # _start_position_idx_df = pd.DataFrame(columns=['pose_index'])
        # _start_position_idx_df.loc[-1] = [i]
        # _start_position_idx_df.index = _start_position_idx_df.index + 1
        # _start_position_idx_df.to_hdf(files_start_position_idx[i], key='data', format='fixed')

        start_pose_idx += 1
    # tf_vecs = []
    # dts = []
    # for i in range(len(files_tf)):
    #     file_date = files_tf[i].split('__')[-3].replace('_0', '')
    #     d = dt.datetime.strptime(file_date, "%Y%m%d_%H-%M-%S")
    #     dts.append(d)

    #     rotation_started = pd.read_hdf(files_rotation_started[i])
    #     if rotation_started.empty:
    #         print(i)
    #         continue

    #     rotation_started_ts = pd.read_hdf(files_rotation_started[i]).at[0, 'rotation_event_ts']
    #     tf_df = pd.read_hdf(files_tf[i])
    #     tf_static_df = pd.read_hdf(files_tf_static[i])
    #     joint_states_df = pd.read_hdf(files_joint_states[i])

    #     tf_df = pb.get_tf_df_at_closest_timestamp(tf_df=tf_df, tf_static_df=tf_static_df, timestamp=rotation_started_ts)
    #     # print(tf_df)

    #     tf_tof_to_base = pb.get_tf_matrix_from_df(
    #         target_frame=f"amiga__base",
    #         source_frame=f"mock_pruner__tool0",
    #         tf_df=tf_df,
    #     )
    #     vec = tf_tof_to_base @ [0, 0, 0, 1]

    #     tf_vecs.append(vec)

    fig = go.Figure()
    start_points = np.asarray(start_points)

    # matches = np.all(np.isclose(gen_pts[:, None], start_points[None,:], atol=0.01), axis=2)
    # indices_in_gen_pts = np.where(matches.any(axis=1))[0]
    # print(indices_in_gen_pts)
    # print(np.diff(indices_in_gen_pts))
    # import sys
    # sys.exit()

    start_points_idxs = np.asarray(start_points_idxs)
    # pp.pprint(start_points)
    # tf_vecs = np.asarray(tf_vecs)
    tf_points = np.asarray(tf_points)
    # ids = np.concat([[43,44,45,46,47,48,49], [52, :]])
    dts = np.asarray(dts)
    # pp.pprint(tf_vecs)
    fig.add_trace(
        go.Scatter3d(
            x=start_points[:, 0],
            y=start_points[:, 1],
            z=start_points[:, 2],
            mode="markers",
            marker=dict(size=4),
            name="start_pts",
            customdata=np.column_stack((start_points_idxs.reshape(-1, 1), np.asarray(dts).reshape(-1, 1))),
            hovertemplate="ID: %{customdata[0]}<br>date: %{customdata[1]}<br>x:%{x:.3f}<br>y:%{y:.3f}<br>z:%{z:.3f}",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=gen_pts[:, 0],
            y=gen_pts[:, 1],
            z=gen_pts[:, 2],
            mode="markers",
            marker=dict(size=4),
            name="tf_pts",
        )
    )
    fig.show()
    # # fig.add_trace(
    # #     go.Scatter3d(
    # #         x=tf_vecs[:, 0],
    # #         y=tf_vecs[:, 1],
    # #         z=tf_vecs[:, 2],
    # #         mode='markers',
    # #         marker=dict(size=4),
    # #         name="tf_vecs",
    # #         # customdata=np.column_stack((js.reshape(-1,1),)),
    # #         # hovertemplate='ID: %{customdata[0]}<br>x:%{x:.3f}<br>y:%{y:.3f}<br>z:%{z:.3f}<extra></extra>'
    # #     )
    # # )
    # fig.update_layout(scene=dict(aspectmode='data'))
    # fig.show()

    return


if __name__ == "__main__":
    main()
